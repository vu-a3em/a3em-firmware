// Header Inclusions ---------------------------------------------------------------------------------------------------

#include <stdint.h>
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "static_config.h"
#include "nn_model_data.h"
#include "nn.h"


// C++-Specific Type and Function Definitions --------------------------------------------------------------------------

struct nn_model {
   uint8_t *arena;
   uint32_t arena_size;
   const tflite::Model *model;
   tflite::MicroInterpreter *interpreter;
   TfLiteTensor *model_input, *model_output;
};

void operator delete(void *p, size_t) {}


// Static Global Variables ---------------------------------------------------------------------------------------------

static struct nn_model model;
static constexpr uint32_t tensor_arena_size = nn_model_LEN + (sizeof(uint32_t) * AI_NUM_OUTPUT_FEATURES) + (sizeof(float) * AI_NUM_INPUT_FEATURES);
__attribute__((section(".ai"))) static uint8_t tensor_arena[tensor_arena_size];


// Public API Functions ------------------------------------------------------------------------------------------------

extern "C" bool nn_initialize(void)
{
   // Initialize model working area
   model.arena = tensor_arena;
   model.arena_size = tensor_arena_size;

   // Initialize the TFLite interpreter and map the AI model data into a usable structure
   tflite::InitializeTarget();
   model.model = tflite::GetModel(nn_model);
   if (model.model->version() != TFLITE_SCHEMA_VERSION)
      return false;

   // Resolve all necessary model layers
   static tflite::MicroMutableOpResolver<7> resolver;
   resolver.AddQuantize();
   resolver.AddReshape();
   resolver.AddConv2D();
   resolver.AddTranspose();
   resolver.AddLeakyRelu();
   resolver.AddFullyConnected();
   resolver.AddDequantize();

   // Build a TFLite interpreter to use for inference
   static tflite::MicroInterpreter static_interpreter(model.model, resolver, model.arena, model.arena_size);
   model.interpreter = &static_interpreter;

   // Allocate memory for the model's input and output tensors
   if (model.interpreter->AllocateTensors() != kTfLiteOk)
      return false;
   model.model_input = model.interpreter->input(0);
   model.model_output = model.interpreter->output(0);
   return true;
}

extern "C" float* nn_invoke(float *input)
{
   // Copy data into the model's input tensor, invoke the model, and return the output
   memcpy(model.model_input->data.int8, input, model.model_input->bytes);
   model.interpreter->Invoke();
   return model.model_output->data.f;
}
