#ifndef VEHICLEINTERFACE_H_INCLUDED
#define VEHICLEINTERFACE_H_INCLUDED

#include <mutex>

class VehicleInterface
{
public:
    VehicleInterface(){}
    ~VehicleInterface(){}

    bool getBrakePedal()
    {
        std::unique_lock<std::mutex> lock(matrixMutex);
        return brakePedal; // return a copy of the matrix
    }

    void setBrakePedal(const bool brakePedal_ref)
    {
        std::unique_lock<std::mutex> lock(matrixMutex);
        brakePedal = brakePedal_ref;
    }

    bool getGasPedal()
    {
        std::unique_lock<std::mutex> lock(matrixMutex);
        return gasPedal; // return a copy of the matrix
    }

    void setGasPedal(const bool gasPedal_ref)
    {
        std::unique_lock<std::mutex> lock(matrixMutex);
        gasPedal = gasPedal_ref;
    }

    int getSteeringWheel()
    {
        std::unique_lock<std::mutex> lock(matrixMutex);
        return steeringWheel; // return a copy of the matrix
    }

    void setSteeringWheel(const int steeringWheel_ref)
    {
        std::unique_lock<std::mutex> lock(matrixMutex);
        steeringWheel = steeringWheel_ref;
    }

private:

    bool brakePedal, gasPedal;
    std::mutex matrixMutex;
    int steeringWheel;
};

#endif // VEHICLEINTERFACE_H_INCLUDED
