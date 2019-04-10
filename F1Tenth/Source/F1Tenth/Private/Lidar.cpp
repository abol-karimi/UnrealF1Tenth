// Fill out your copyright notice in the Description page of Project Settings.

#include "Lidar.h"

// Sets default values
ALidar::ALidar()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void ALidar::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void ALidar::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

