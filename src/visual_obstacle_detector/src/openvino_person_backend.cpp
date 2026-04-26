#include "visual_obstacle_detector/openvino_person_backend.h"

#include <algorithm>
#include <cstring>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <openvino/openvino.hpp>

namespace
{

template<typename Container>
std::string joinStrings(const Container& values, const std::string& delimiter)
{
  std::ostringstream stream;
  bool first = true;
  for (const auto& value : values) {
    if (!first) {
      stream << delimiter;
    }
    first = false;
    stream << value;
  }
  return stream.str();
}

std::string shapeToString(const ov::Shape& shape)
{
  std::ostringstream stream;
  stream << "[";
  for (std::size_t i = 0; i < shape.size(); ++i) {
    if (i > 0) {
      stream << ", ";
    }
    stream << shape[i];
  }
  stream << "]";
  return stream.str();
}

std::size_t shapeElementCount(const ov::Shape& shape)
{
  std::size_t count = 1;
  for (const std::size_t dim : shape) {
    count *= dim;
  }
  return count;
}

void copyErrorMessage(const std::string& message, char* error_buffer, const std::size_t error_buffer_size)
{
  if (error_buffer == nullptr || error_buffer_size == 0U) {
    return;
  }

  const std::size_t copy_size = std::min(message.size(), error_buffer_size - 1U);
  std::memcpy(error_buffer, message.data(), copy_size);
  error_buffer[copy_size] = '\0';
}

}  // namespace

struct PersonOpenVinoBackend
{
  ov::Core core;
  ov::CompiledModel compiled_model;
  ov::InferRequest infer_request;
  ov::Shape input_shape;
  ov::Shape output_shape;
  std::vector<float> output_buffer;
  std::string summary;
};

extern "C" PersonOpenVinoBackend* person_openvino_backend_create(
  const char* model_path,
  const int input_width,
  const int input_height,
  const char* device_name,
  char* error_buffer,
  const std::size_t error_buffer_size)
{
  try {
    if (model_path == nullptr || model_path[0] == '\0') {
      throw std::runtime_error("OpenVINO model path is empty");
    }
    if (device_name == nullptr || device_name[0] == '\0') {
      throw std::runtime_error("OpenVINO device request is empty");
    }
    if (input_width <= 0 || input_height <= 0) {
      throw std::runtime_error("OpenVINO input shape is invalid");
    }

    std::unique_ptr<PersonOpenVinoBackend> backend(new PersonOpenVinoBackend());

    std::ostringstream summary;
    const std::vector<std::string> available_devices = backend->core.get_available_devices();
    if (available_devices.empty()) {
      summary << "OpenVINO reported no available devices for person detector\n";
    } else {
      for (const std::string& device_name_it : available_devices) {
        std::string full_name = "(unknown)";
        try {
          full_name = backend->core.get_property(device_name_it, ov::device::full_name);
        } catch (const ov::Exception&) {
        }
        summary << "OpenVINO device available for person detector: "
                << device_name_it
                << " | full_name="
                << full_name
                << "\n";
      }
    }

    summary << "Loading OpenVINO person model from " << model_path << " with device request " << device_name << "\n";

    std::shared_ptr<ov::Model> model = backend->core.read_model(model_path);
    if (!model) {
      throw std::runtime_error("OpenVINO failed to create a model object");
    }
    if (model->inputs().empty() || model->outputs().empty()) {
      throw std::runtime_error("OpenVINO model for person detector has no inputs or outputs");
    }

    const ov::Output<ov::Node> input_port = model->input();
    model->reshape({
      {
        input_port.get_any_name(),
        ov::PartialShape{1, 3, input_height, input_width},
      }
    });

    ov::AnyMap properties;
    properties[ov::hint::performance_mode.name()] = ov::hint::PerformanceMode::LATENCY;

    backend->compiled_model = backend->core.compile_model(model, device_name, properties);
    backend->infer_request = backend->compiled_model.create_infer_request();
    backend->input_shape = backend->compiled_model.input().get_shape();
    backend->output_shape = backend->compiled_model.output().get_shape();
    backend->output_buffer.resize(shapeElementCount(backend->output_shape));

    std::vector<std::string> execution_devices;
    try {
      execution_devices = backend->compiled_model.get_property(ov::execution_devices);
    } catch (const ov::Exception&) {
    }
    summary << "OpenVINO person detector execution devices: "
            << (execution_devices.empty() ? "(unknown)" : joinStrings(execution_devices, ", "))
            << "\n";
    summary << "OpenVINO person detector tensors: input="
            << shapeToString(backend->input_shape)
            << " output="
            << shapeToString(backend->output_shape);

    backend->summary = summary.str();
    return backend.release();
  } catch (const std::exception& ex) {
    copyErrorMessage(ex.what(), error_buffer, error_buffer_size);
    return nullptr;
  }
}

extern "C" void person_openvino_backend_destroy(PersonOpenVinoBackend* backend)
{
  delete backend;
}

extern "C" const char* person_openvino_backend_get_summary(const PersonOpenVinoBackend* backend)
{
  if (backend == nullptr) {
    return "";
  }
  return backend->summary.c_str();
}

extern "C" bool person_openvino_backend_infer(
  PersonOpenVinoBackend* backend,
  const float* input_data,
  const std::size_t input_element_count,
  PersonOpenVinoTensorView* output_view,
  char* error_buffer,
  const std::size_t error_buffer_size)
{
  try {
    if (backend == nullptr) {
      throw std::runtime_error("OpenVINO backend is not initialized");
    }
    if (input_data == nullptr) {
      throw std::runtime_error("OpenVINO input tensor is null");
    }
    if (output_view == nullptr) {
      throw std::runtime_error("OpenVINO output view is null");
    }

    const std::size_t expected_input_count = shapeElementCount(backend->input_shape);
    if (input_element_count != expected_input_count) {
      std::ostringstream stream;
      stream << "OpenVINO input tensor size mismatch, expected "
             << expected_input_count
             << " elements but got "
             << input_element_count;
      throw std::runtime_error(stream.str());
    }

    ov::Tensor input_tensor(
      ov::element::f32,
      backend->input_shape,
      const_cast<float*>(input_data));
    backend->infer_request.set_input_tensor(input_tensor);
    backend->infer_request.infer();

    const ov::Tensor output_tensor = backend->infer_request.get_output_tensor();
    backend->output_shape = output_tensor.get_shape();
    const std::size_t output_count = shapeElementCount(backend->output_shape);
    backend->output_buffer.resize(output_count);
    const float* output_data = output_tensor.data<const float>();
    std::copy(output_data, output_data + output_count, backend->output_buffer.begin());

    if (backend->output_shape.size() != 3U) {
      std::ostringstream stream;
      stream << "OpenVINO output rank must be 3, got " << shapeToString(backend->output_shape);
      throw std::runtime_error(stream.str());
    }

    output_view->data = backend->output_buffer.data();
    output_view->dim0 = backend->output_shape[0];
    output_view->dim1 = backend->output_shape[1];
    output_view->dim2 = backend->output_shape[2];
    return true;
  } catch (const std::exception& ex) {
    copyErrorMessage(ex.what(), error_buffer, error_buffer_size);
    return false;
  }
}
