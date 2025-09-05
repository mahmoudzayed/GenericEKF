#pragma once

#include <IMotionModel.hpp>
#include <data_types.hpp>
#include <memory>
#include <unordered_map>
#include <stdexcept>


class MotionModelHandler : public IMotionModel
{
private:
    static inline std::unordered_map<MotionModelType, std::shared_ptr<IMotionModel>> MotionModelDB_;
    std::shared_ptr<IMotionModel> currentModel;
    std::shared_ptr<IMotionModel> nextModel;
    MotionModelType currentType_;


public:
    MotionModelHandler(MotionModelType defaultModel) : currentType_(defaultModel) {
        if (MotionModelDB_.find(defaultModel) != MotionModelDB_.end())
        {
            currentModel = MotionModelDB_[defaultModel];
        }
        else
        {
            throw std::invalid_argument("Default model not found in MotionModelDB_");
        }
    }

    static void registerModel(MotionModelType type, std::shared_ptr<IMotionModel> model) {
        if (MotionModelDB_.find(type) == MotionModelDB_.end())
        {
            MotionModelDB_[type] = std::move(model);
        }
        else
        {
            throw std::runtime_error("Model already registered");
        }
    }

    void setNextModel(MotionModelType nextModelType) {
        auto it = MotionModelDB_.find(nextModelType);
        if (it != MotionModelDB_.end())
        {
            nextModel = it->second;
        }
        else
        {
            throw std::invalid_argument("Next model not found in MotionModelDB_");
        }
    }

    void switchToNextModel() {
        if (nextModel) {
            currentModel = nextModel;
            nextModel.reset();
        } else {
            throw std::runtime_error("Next model is not set");
        }
    }

private:
    MotionModelType getTypeOfModel(std::shared_ptr<IMotionModel> model);
};
