// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include <boost/polygon/polygon.hpp>
#include <vector>

typedef double coordinate_type;
typedef boost::polygon::point_data<coordinate_type> point_type;
typedef boost::polygon::segment_data<coordinate_type> segment_type;

/**
 * 
 */
class F1TENTH_API VoronoiGraph
{
public:
	VoronoiGraph();
	~VoronoiGraph();
	void MakeRoadmap(const std::vector<segment_type>& Walls);
	void GetPlan(std::vector<segment_type>& OutPlan);

private:
	bool get_trackopening(point_type& OutTrackOpening, const std::vector<segment_type>& Walls, double min_gap);

};

