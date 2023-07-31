# Line of Sight for Communication Channels between Swarms

This project aims to design a Line of Sight (LOS) communication channel between two swarms of IoT devices (Unmanned Aerial Vehicles - UAVs) using C++ and ArduPilot. The communication channel will be enhanced with the integrated Doppler effect by calculating the distance between two UAVs and utilizing antenna gain.

## Introduction
It presents a comprehensive study of the UAV swarm communication model, focusing on the receive signal strength (RSS) and power loss modeling using UCT algorithm. The mathematical representation of the channel matrix is discussed. The algorithm is applied to predict swarm clusters, and the dendrogram of all types is investigated to obtain the correct swarm formation. 

## Mathematical Model for Swarm Control
The mathematical model for swarm control is formulated for a unit swarm  with a 2D flying pattern assumption. The model considers the movement parameters of the master and slave UAVs and ensures safe distance to avoid collisions.

## Swarm Formation
The project includes the generation of datasets for five different types of triangular swarm formations, including equilateral, isosceles, obtuse, acute, and right swarm formations.
