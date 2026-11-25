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

// Adapter for raw-input models (e.g., GNN_8obs.onnx, GATR_4obs.onnx, xarm6/onnx_models/GATR.onnx):
//   raw_inputs [1,N] -> probabilities [1,M], all float32, where N and M are read from the model
//   (e.g., 192 -> 16 for planar 2-DOF, and 5088 -> 144 for xArm6, with a three-layer pattern tree).
// The model constructs its graph features and applies grouped softmax internally.
// Raw values are passed unchanged; no external scaling or preprocessing is used.
// Construct once. One instance is intended for one calling thread.
class OnnxCNN {
public:
    using Probabilities = std::vector<float>;

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
        raw_buffer_.resize(input_count_);
        probabilities_.resize(output_count_);

        const std::array<int64_t, 2> input_shape{1, static_cast<int64_t>(input_count_)};
        const std::array<int64_t, 2> output_shape{1, static_cast<int64_t>(output_count_)};
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

    std::size_t inputCount() const { return input_count_; }
    std::size_t outputCount() const { return output_count_; }

    // Original CSV order (see SL_DRGBT::generateInput): start/goal coordinates, spine lengths,
    // link clearances, then OR-vector components (e.g., 4 + 52 + 34 + 102 = 192 for planar 2-DOF).
    // The returned reference is overwritten by the next predict() call.
    template<class Number>
    const Probabilities& predict(const Number* raw, std::size_t count) {
        static_assert(std::is_floating_point<Number>::value,
                      "Use floating-point input values.");
        if (raw == nullptr || count != input_count_) {
            throw std::invalid_argument(
                "Expected exactly " + std::to_string(input_count_) + " raw input values; received " +
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
    // Returns N for a float32 tensor of shape [1,N].
    static std::size_t checkTensor(const Ort::TypeInfo& type, const std::string& name) {
        if (type.GetONNXType() != ONNX_TYPE_TENSOR) {
            throw std::runtime_error(name + " must be a tensor.");
        }
        const auto tensor = type.GetTensorTypeAndShapeInfo();
        const std::vector<int64_t> shape = tensor.GetShape();
        if (tensor.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
            shape.size() != 2 || shape[0] != 1 || shape[1] < 1) {
            throw std::runtime_error("Unexpected shape/type for " + name +
                                     "; expected float32 [1,N]. Check the exported model interface.");
        }
        return static_cast<std::size_t>(shape[1]);
    }

    void validateInterface() {
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
        input_count_ = checkTensor(session_.GetInputTypeInfo(0), input);
        output_count_ = checkTensor(session_.GetOutputTypeInfo(0), output);
    }

    Ort::Env env_;
    Ort::SessionOptions options_;
    Ort::Session session_;
    Ort::MemoryInfo memory_;
    std::size_t input_count_{0};
    std::size_t output_count_{0};
    std::vector<float> raw_buffer_{};
    Probabilities probabilities_{};
    Ort::Value input_tensor_;
    Ort::Value output_tensor_;
};

// Public API: N raw values -> M grouped probabilities (N and M are given by the model).
// Initialize once, reuse for every sample. One instance per calling thread.
class Predictor {
public:
    using Probabilities = OnnxCNN::Probabilities;

    explicit Predictor(const std::string& onnx_model_path, int intra_op_threads = 1)
        : cnn_(onnx_model_path, intra_op_threads) {}

    // Compatibility for existing planner callers. Feature metadata is unused
    // because raw-input models contain feature construction themselves.
    [[deprecated("Pass only the raw-input model path; feature metadata is unused.")]]
    Predictor(const std::string& onnx_model_path, const std::string&,
                    int intra_op_threads = 1)
        : Predictor(onnx_model_path, intra_op_threads) {}

    std::size_t getInputCount() const { return cnn_.inputCount(); }
    std::size_t getOutputCount() const { return cnn_.outputCount(); }

    template<class Number>
    Probabilities predict(const Number* raw, std::size_t count) {
        return cnn_.predict(raw, count); // Return a copy of the M values.
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
