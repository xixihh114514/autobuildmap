#pragma once

#include <cstddef>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct PersonOpenVinoBackend PersonOpenVinoBackend;

typedef struct PersonOpenVinoTensorView {
  const float* data;
  std::size_t dim0;
  std::size_t dim1;
  std::size_t dim2;
} PersonOpenVinoTensorView;

PersonOpenVinoBackend* person_openvino_backend_create(
  const char* model_path,
  int input_width,
  int input_height,
  const char* device_name,
  char* error_buffer,
  std::size_t error_buffer_size);

void person_openvino_backend_destroy(PersonOpenVinoBackend* backend);

const char* person_openvino_backend_get_summary(const PersonOpenVinoBackend* backend);

bool person_openvino_backend_infer(
  PersonOpenVinoBackend* backend,
  const float* input_data,
  std::size_t input_element_count,
  PersonOpenVinoTensorView* output_view,
  char* error_buffer,
  std::size_t error_buffer_size);

#ifdef __cplusplus
}
#endif
