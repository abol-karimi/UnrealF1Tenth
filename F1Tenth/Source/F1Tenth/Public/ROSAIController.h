// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "ROSIntegration/Public/std_msgs/Float32.h"


#include "F1TenthPawn.h"
#include "CoreMinimal.h"
#include "AIController.h"
#include "ROSAIController.generated.h"

/**
 * 
 */
UCLASS()
class F1TENTH_API AROSAIController : public AAIController
{
	GENERATED_BODY()
	
	virtual void BeginPlay() override;

private:
	AF1TenthPawn* ControlledVehicle = nullptr;
	UROSIntegrationGameInstance* rosinst;
	UTopic *MotorDutyCycle;
	UTopic *ServoPosition;
	std::function<void(TSharedPtr<FROSBaseMsg>)> DutyCycleCallback = [this](TSharedPtr<FROSBaseMsg> msg) -> void
    {
        auto Concrete = StaticCastSharedPtr<ROSMessages::std_msgs::Float32>(msg);
        if (Concrete.IsValid())
        {
			float duty_cycle = Concrete->_Data;
            UE_LOG(LogTemp, Log, TEXT("Duty cycle was: %f"), duty_cycle);
			this->ControlledVehicle->MoveForward(duty_cycle);
        }
        return;
    };
	std::function<void(TSharedPtr<FROSBaseMsg>)> ServoPositionCallback = [this](TSharedPtr<FROSBaseMsg> msg) -> void
    {
        auto Concrete = StaticCastSharedPtr<ROSMessages::std_msgs::Float32>(msg);
        if (Concrete.IsValid())
        {
			float servo_position = 2 * Concrete->_Data - 1;
            UE_LOG(LogTemp, Log, TEXT("Servo position was: %f"), servo_position);
			this->ControlledVehicle->MoveRight(servo_position);
        }
        return;
    };
};
