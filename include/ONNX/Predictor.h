#pragma once

#include <onnxruntime_cxx_api.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

// Adapter for GNN_8obs.onnx:
//   raw_inputs [1,192] -> probabilities [1,16], all float32.
// The model constructs its graph features and applies grouped softmax internally.
// Raw values are passed unchanged; no external scaling or preprocessing is used.
// Construct once. One instance is intended for one calling thread.
class OnnxCNN {
public:
    static constexpr std::size_t InputCount = 192;
    static constexpr std::size_t OutputCount = 16;
    using Probabilities = std::array<float, OutputCount>;

    explicit OnnxCNN(const std::string& onnx_model_path, int intra_op_threads = 1)
        : env_(ORT_LOGGING_LEVEL_WARNING, "RPMPLv2_CNN"),
          session_(nullptr),
          memory_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)),
          input_tensor_(nullptr),
          output_tensor_(nullptr) {
        if (intra_op_threads < 1) {
            throw std::invalid_argument("Thread count must be positive.");
        }
        options_.SetIntraOpNumThreads(intra_op_threads);
        options_.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
        options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        session_ = Ort::Session(env_, onnx_model_path.c_str(), options_);
        validateInterface();

        const std::array<int64_t, 2> input_shape{1, InputCount};
        const std::array<int64_t, 2> output_shape{1, OutputCount};
        input_tensor_ = Ort::Value::CreateTensor<float>(
            memory_, raw_buffer_.data(), raw_buffer_.size(),
            input_shape.data(), input_shape.size());
        output_tensor_ = Ort::Value::CreateTensor<float>(
            memory_, probabilities_.data(), probabilities_.size(),
            output_shape.data(), output_shape.size());
    }

    OnnxCNN(const OnnxCNN&) = delete;
    OnnxCNN& operator=(const OnnxCNN&) = delete;
    OnnxCNN(OnnxCNN&&) = delete;
    OnnxCNN& operator=(OnnxCNN&&) = delete;

    // Original CSV order: 4 start/goal coordinates, 52 spine lengths,
    // 34 link clearances, then 102 OR-vector components.
    // The returned reference is overwritten by the next predict() call.
    template<class Number>
    const Probabilities& predict(const Number* raw, std::size_t count) {
        static_assert(std::is_floating_point<Number>::value,
                      "Use floating-point input values.");
        if (raw == nullptr || count != InputCount) {
            throw std::invalid_argument(
                "Expected exactly 192 raw input values; received " +
                std::to_string(count) + ".");
        }
        for (std::size_t i = 0; i < count; ++i) {
            const double value = static_cast<double>(raw[i]);
            if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max()) {
                throw std::invalid_argument("Input must be finite and representable as float32.");
            }
            raw_buffer_[i] = static_cast<float>(raw[i]);
        }
        const char* input_names[] = {"raw_inputs"};
        const char* output_names[] = {"probabilities"};
        session_.Run(Ort::RunOptions{nullptr}, input_names, &input_tensor_, 1,
                     output_names, &output_tensor_, 1);
        return probabilities_;
    }

private:
    static void checkTensor(const Ort::TypeInfo& type,
                            const std::vector<int64_t>& expected_shape,
                            const std::string& name) {
        if (type.GetONNXType() != ONNX_TYPE_TENSOR) {
            throw std::runtime_error(name + " must be a tensor.");
        }
        const auto tensor = type.GetTensorTypeAndShapeInfo();
        if (tensor.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
            tensor.GetShape() != expected_shape) {
            throw std::runtime_error("Unexpected shape/type for " + name +
                                     "; check the exported model interface.");
        }
    }

    void validateInterface() const {
        if (session_.GetInputCount() != 1 || session_.GetOutputCount() != 1) {
            throw std::runtime_error("Expected input 'raw_inputs' and output 'probabilities'.");
        }
        Ort::AllocatorWithDefaultOptions allocator;
        const std::string input = session_.GetInputNameAllocated(0, allocator).get();
        const std::string output = session_.GetOutputNameAllocated(0, allocator).get();
        if (input != "raw_inputs") {
            throw std::runtime_error("Unexpected input name: " + input);
        }
        if (output != "probabilities") {
            throw std::runtime_error("Unexpected output name: " + output);
        }
        checkTensor(session_.GetInputTypeInfo(0), {1, InputCount}, input);
        checkTensor(session_.GetOutputTypeInfo(0), {1, OutputCount}, output);
    }

    Ort::Env env_;
    Ort::SessionOptions options_;
    Ort::Session session_;
    Ort::MemoryInfo memory_;
    std::array<float, InputCount> raw_buffer_{};
    Probabilities probabilities_{};
    Ort::Value input_tensor_;
    Ort::Value output_tensor_;
};

// Public API: raw 192 values -> 16 grouped probabilities.
// Initialize once, reuse for every sample. One instance per calling thread.
class Predictor {
public:
    static constexpr std::size_t InputCount = OnnxCNN::InputCount;
    using Probabilities = OnnxCNN::Probabilities;

    explicit Predictor(const std::string& onnx_model_path, int intra_op_threads = 1)
        : cnn_(onnx_model_path, intra_op_threads) {}

    // Compatibility for existing planner callers. Feature metadata is unused
    // because raw-input models contain feature construction themselves.
    [[deprecated("Pass only the raw-input model path; feature metadata is unused.")]]
    Predictor(const std::string& onnx_model_path, const std::string&,
                    int intra_op_threads = 1)
        : Predictor(onnx_model_path, intra_op_threads) {}

    template<class Number>
    Probabilities predict(const Number* raw, std::size_t count) {
        return cnn_.predict(raw, count); // Return a copy of the 16 values.
    }
    template<class Number>
    Probabilities predict(const std::vector<Number>& raw) {
        return predict(raw.data(), raw.size());
    }
    template<class Number, std::size_t Count>
    Probabilities predict(const std::array<Number, Count>& raw) {
        return predict(raw.data(), raw.size());
    }

private:
    OnnxCNN cnn_;
};
