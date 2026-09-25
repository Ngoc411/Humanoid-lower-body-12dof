#include <onnxruntime_cxx_api.h>
#include <iostream>
#include <array>
#include <stdexcept>

static constexpr int OBS_DIM    = 45;
static constexpr int ACTION_DIM = 12;

int main(int argc, char** argv)
{
    if (argc != OBS_DIM + 1) {
        std::cerr << "Usage: test_model_node";
        for (int i = 0; i < OBS_DIM; ++i) std::cerr << " f" << i;
        std::cerr << "\n(expects exactly " << OBS_DIM << " floats)\n";
        return 1;
    }

    std::array<float, OBS_DIM> input;
    try {
        for (int i = 0; i < OBS_DIM; ++i)
            input[i] = std::stof(argv[i + 1]);
    } catch (const std::exception& e) {
        std::cerr << "Failed to parse input: " << e.what() << "\n";
        return 1;
    }

    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "test_model_node");

    Ort::SessionOptions opts;
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    opts.SetIntraOpNumThreads(1);
    opts.SetInterOpNumThreads(1);

    Ort::Session session(env, "/home/ngoc/ros2_ws/src/ws1_cpp/actor.onnx", opts);

    Ort::AllocatorWithDefaultOptions allocator;
    std::string input_name  = session.GetInputNameAllocated(0, allocator).get();
    std::string output_name = session.GetOutputNameAllocated(0, allocator).get();

    std::array<int64_t, 2> shape = {1, OBS_DIM};
    Ort::MemoryInfo mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        mem, input.data(), OBS_DIM, shape.data(), shape.size());

    const char* in_names[]  = { input_name.c_str() };
    const char* out_names[] = { output_name.c_str() };

    auto output_tensors = session.Run(
        Ort::RunOptions{nullptr}, in_names, &input_tensor, 1, out_names, 1);

    const float* out = output_tensors.front().GetTensorData<float>();

    for (int i = 0; i < ACTION_DIM; ++i) {
        std::cout << out[i];
        if (i < ACTION_DIM - 1) std::cout << " ";
    }
    std::cout << "\n";

    return 0;
}
