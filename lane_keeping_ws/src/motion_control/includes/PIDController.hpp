#pragma once

class PIDController
{
    public:
        PIDController(float kp, float ki, float kd);
        PIDController();
        ~PIDController() = default;

        void initializePID(float kp, float ki, float kd);

    private:
        float kp_;
        float ki_;
        float kd_;
};
