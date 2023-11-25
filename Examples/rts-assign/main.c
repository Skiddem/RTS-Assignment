#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include <stdio.h>
#include <float.h>
#include <stdbool.h>
#include <stdlib.h>
#include "queue.h"
#include "supporting_functions.h"
#include "semphr.h"

#define _CRT_SECURE_NO_WARNINGS
#define V 40 //number of malls(nodes)
#define totalQueries 10000
double travellingTimeMatrix[V][V];
double shortestDistanceMatrix[V][V];
QueueHandle_t userQueue;
QueueHandle_t responseQueue;
SemaphoreHandle_t queueSemaphore;

typedef struct {

	int noQueries;
}Queries;

typedef struct {
	int userID;
	int speed;
	int src;
	int destination;
	TickType_t elapsedTime, startTime;
}User;

typedef struct {
	unsigned long minElapsedTime;
	unsigned long maxElapsedTime;
	unsigned long totalElapsedTime;
	double avgElapsedTime;
}Response;

//shopping malls
char* Malls[V] = {
	"Berjaya Times Square", "Bukit Bintang Plaza", "Fahrenheit 88", "Imbi Plaza", "Lot 10", "Mitsui Shopping Park Lalaport Kl", "Plaza Low Yat", "Pavilion KL", "Starhill Gallery", "Sungei Wang Plaza",
	"Avenue K Shopping Mall", "Intermark Mall", "Suria KLCC", "LINC KL", "The Weld", "Great Eastern Shopping Centre", "Pertama Complex", "Quill City Mall", "Sogo KL", "Sunway Putra Mall", "CapSquare Centre",
	"Pudu Plaza", "Kenanga Wholesale City", "Plaza Salak Park", "Bangsar Shopping Centre", "Bangsar Village I and II", "Pavivilion BJ", "NU Sentral", "Shamelin Shopping Centre", "ATMSC", "IKEA Cheras",
	"MyTown Shopping Centre", "Sunway Velocity Mall", "Viva Home", "Damansara City Mall", "Glo Damansara", "Hartamas Shopping Centre", "Publika Shopping Gallery", "Plaza OUG", "Pearl Point Shopping Centre"
};

double distanceMatrix[V][V] = {

		{0, 2.1 ,2.3 ,1.8 ,2.6 ,0.35 ,1.9 ,2.8 ,2.5 ,1.8 ,4.1 ,7.2 ,3.7 ,6.2 ,2.8 ,7.1 ,5.1 ,5.8 ,4.9 ,7.1 ,4.3 ,1.3 ,1.3 ,9.9 ,10.4 ,8.7 ,17.6 ,6.4 ,5.4 ,3.4 ,2.6 ,2.9 ,2.7 ,3.4 ,7.9 ,14.5 ,11.1 ,12.7 ,10.5 ,12.3},//berjaya
		{0.65,0,1.1,0.3,1.5,1,0.17,1.5,1.4,0.15,2.8,3.7,2.4,3.8,1.5,8.1,3.9,5.2,3.8,6.8,3,1.7,1.9,9.4,8,6.5,17.1,5.4,5.3,4.6,3.9,3.5,3.2,4,6.7,12.3,9.7,8,11.1,10.2},//bbplaza
		{0.8,0.3,0,0.6,0.94,1.1,0.45,0.85,0.27,0.45,2.2,4.1,1.9,3.2,0.95,5.8,2.8,4.1,3.1,8,2.4,1.5,2,9.4,8.2,6.7,17.2,5.7,5.4,4.6,4,3.5,3.4,4.2,8.6,14.2,9.9,8.4,11.2,10.3}, //f88
		{0.27,0.5,0.8,0,1.2,0.6,0.81,1.1,1,0.16,2.4,4.4,2.2,3.4,1.1,7.8,3,4.3,3.3,10.2,2.6,1.5,1.5,9,9.3,7.7,16.7,5.7,4.9,4.2,3.5,3.1,3,3.7,8.1,13.7,11.1,9.4,10.7,9.8},//ImbiPLaza
		{0.6,0.21,0.5,0.5,0,1,0.35,0.8,0.7,0.35,2.2,4,1.8,3.1,0.85,8,2.7,4,3,7.9,2.3,1.8,1.9,9.3,9.7,6.6,17,5.6,5.2,4.5,3.8,3.4,3.3,4,8.5,14,11.5,8.3,11.1,10.2},//lot10
		{4,4.3,4.6,3.7,5.8,0,3.8,5,5.5,3.8,7.7,6.8,6.9,5.9,5,10.8,6.7,8,6.6,9.6,5.9,2.6,0.9,8.1,8.7,7,15.8,5.1,7.5,5.2,3.1,3.4,5.1,3,7.5,13,10.5,8.8,10.1,9.2},//MSPL
		{0.4,0.6,0.9,0.08,1.3,0.75,0,1.2,1.1,0.02,2.5,4.5,2.2,3.5,1.2,7.9,3.1,5.1,3.6,6.6,2.7,1.5,1.7,9.1,7.8,6.3,16.9,5.3,5.1,4.3,3.7,3.3,3,3.8,6.6,12.1,9.6,9.2,10.9,10},//lowyat
		{1.3,1,0.55,1.3,0.75,1.6,1.4,0,0.45,1.3,4.6,3.7,3.8,2.8,1.6,5.4,3.5,8.7,3.8,7.6,3.1,2,2.5,9.9,10.4,8.7,17.6,6.7,5.9,5.1,4.4,4,4.2,4.7,9.1,14.7,12.1,11.8,11.7,10.8},//PavKL
		{0.95,0.5,0.13,0.85,0.3,1.3,0.7,1.1,0,0.65,2.4,4.3,2.1,3.4,1.2,6.1,3,4.3,3.3,6.1,2.6,2,2.2,9.6,10,6.9,17.3,5.9,5.5,4.8,4.1,3.7,3.6,4.3,8.8,14.3,9.4,8.6,11.4,10.5},//SHGallery
		{0.5,0.3,0.65,0.01,1,0.85,0.02,0.95,0.85,0,2.4,4.2,2,3.3,1,8,2.8,4.1,3.2,5.9,2.4,1.6,1.8,9.2,9.6,6.4,17,5.4,5.2,4.4,3.8,3.4,3.1,3.9,8.3,13.9,9.2,8.4,10.9,10},//SWP
		{5,3.8,3.4,3.9,3.6,5.4,4,2.3,3.3,3.9,0,1.6,1,1,2.2,3.4,2.8,4.7,3.1,5.4,2.3,4.7,6.3,11.1,11.6,12.1,18.8,11,6.8,6.1,4.3,4.1,5.4,5.7,10.4,15.9,10.4,9.7,18.6,13.2},//AveK
		{4.5,3.2,2.8,3.3,3,4.8,3.4,2.4,2.7,3.3,1.7,0,1.2,0.4,3.4,2.8,4.1,4,4.4,4.7,3,4.1,5.7,10.5,10.9,11.3,18.2,10.2,6.2,5.5,3.7,3.5,4.8,5.1,9.7,15.2,9.7,8.9,18,12.6},//Intermark
		{2.4,2,1.6,2.4,1.8,2.7,2.5,1.2,1.5,2.3,0.9,1.8,0,1.5,1.3,3.8,2.5,3.8,2.8,7.3,2.1,3.1,3.6,11,10.2,8.9,18.7,7.8,6.9,6.2,4.5,4.2,5.3,5.7,9,14.5,8.8,8.1,12.8,11.9},//Suria
		{4.1,2.9,2.5,3,2.7,4.5,3.1,2,2.4,2.9,1.9,1.3,1.3,0,2.9,3.2,4.5,4.4,3.8,5.1,3.1,3.8,5.4,10.2,11.3,11.7,17.9,10.7,5.8,5.1,3.4,3.2,4.5,4.7,10.1,15.6,10.1,9.4,17.6,12.2},//linc
		{2.1,1.8,1.4,2.7,1.5,2.4,1.2,0.9,1.2,1.3,1.6,2.5,1.2,3.2,0,5.9,2.2,3.4,2.5,5.2,1.8,2.8,3.3,10.7,7.6,6.1,18.5,5.1,6.7,5.9,4.2,4.5,5,5.7,6.4,11.9,8.5,7.8,10.3,9.4},//weld
		{5.7,4.5,4.1,4.6,4.3,6.1,4.7,4.5,4,4.6,4,3,3.4,2.5,6.5,0,7.3,6.5,7.6,7.2,6.9,5.4,7,11.8,13.4,13.8,20.4,12.8,6.6,6.7,5,4.8,6.1,6.4,12.2,22.4,17.4,12.9,20.1,16.3},//GESC
		{3.6,3.5,3.1,2.9,3.2,3.9,3,2.6,2.9,3,2.2,3.7,2.1,4.2,2.6,6.5,0,1.3,0.3,2.2,1.2,3.6,5.3,10.8,6.3,5.8,18.5,4.7,10.1,7.8,7.5,7.3,7.7,5.7,5.1,10.6,6.9,6.1,10.4,9.5},//pertama
		{3.3,3,2.6,3.3,2.8,3.7,2.8,2.1,2.5,3.3,1.7,3,1.6,3.4,1.4,5.8,1.4,0,1.8,2.6,1.3,7.1,4.6,13.5,9.1,9.5,21.3,8.5,7.9,8.5,6.8,6.5,7.8,8.1,7.9,13.4,7.8,7,12.4,11.5},//quill
		{3.8,5,3.7,3.2,3.9,4.2,3.3,3.3,3.6,3.3,2.9,4.2,2.8,4.6,2.6,11.1,0.4,1.2,0,2.1,1.6,3.9,5.2,10.7,6.2,57,18.4,4.6,10,7.7,10.1,6.5,7.6,5.6,5,10.5,6.8,6.1,10.3,9.4},//sogo
		{6.5,6.8,4.8,6.2,5,5.9,5,4.3,4.7,5.1,3.9,5,3.8,5.6,3.6,8.6,2.7,2.2,3.4,0,3.9,5.7,7,15.1,8.6,7.5,20.2,7.5,11.8,9.5,8.7,8.5,9.4,7.4,6.9,12.9,6.1,5.3,12.1,11.2},//sunwayP
		{3.7,3,2.6,3.1,2.8,4.1,3,2.1,2.5,3.1,1.8,3.3,1.7,3.7,2.2,6.8,1.4,2,1.7,4.4,0,3.8,5,12.7,9,9.5,20.5,8.4,8.3,5.9,7.1,6.8,9.7,7.6,7.8,13.3,7.7,7,12.4,11.5},//capsquare
		{1.6,2.9,2.7,2.3,3,1.1,2.4,3.5,2.9,2.4,5.5,4.6,4.7,3.7,3.3,5.6,4.4,5.7,4.2,10.6,3.6,0,2,9.2,8.4,8.1,16.9,6.1,6.2,2.8,1.2,1.5,2.1,2.7,7.2,12.7,10.2,9.8,13.7,10.7},//pudu
		{3,2.5,4.1,1.9,5,2.1,2,3.2,4.7,2,6.9,6.1,6.1,5.1,3.2,7,4.3,5.1,4.1,8.9,4.7,1.5,0,8,8.3,7.1,15.8,6,7.4,3.2,2.4,2.7,2.5,2.9,7,12.6,10.1,8.4,12.6,10.8},//KWC
		{9.3,9.6,10.4,9.3,10.8,9.6,9.4,11.3,10.6,9.3,12.2,11.4,11.4,10.4,10.3,16.5,11.2,12.5,11,14.6,11.8,10.6,10.5,0,11.9,10.2,8.2,7.8,11.9,9.6,8.8,12,7.6,10.4,12.5,13.8,14.7,13.8,8,7.7},//PLP
		{8.6,8.9,9.3,8.4,9.6,9,8.4,8.8,9.2,8.4,10.4,11.3,10.3,11.7,7.9,16.7,7.4,8.7,7,8.5,7.5,9,9.8,11.1,0,1.8,13.7,4.3,13.4,11.2,10.2,13.9,11,10.1,1.7,5.5,6.4,5.4,10.5,9.6},//BSC
		{7.5,7.9,8.2,7.3,7.9,7.9,7.3,7.2,7.6,7.4,9.3,11.5,9.2,11.9,6.3,15.7,6.3,7.6,6.2,9.3,6.7,7.9,8.7,11.5,2.1,0,13.5,3.2,12.4,10.1,10.8,12.8,10,9,3.7,7,7.9,7.4,9.3,7.9},//BVillage
		{16.8,17.1,17.9,16.8,15.8,17.1,16.9,18.8,15.5,16.8,19.8,18.9,18.9,17.9,17.8,17.9,19.7,20,18.6,22.1,19.3,14.3,18,9.2,19.4,17.7,0,15.3,15.7,16.1,14.2,14,16,17.5,20.2,20.4,22.2,21.3,4.2,5.2},//PavBJ
		{4,4.3,5.4,3.7,5,4.8,3.8,4.3,4.7,3.8,4.9,8.2,4.5,6.3,3.4,12.6,3.9,5.2,3.7,8.7,3.3,4.4,4.5,9.4,6.1,4.4,14.8,0,9.8,7.5,7.5,7.1,7.4,4.9,6.7,12.9,9.6,7.9,10.6,8.7},//NuSen
		{4.6,4.9,5.7,4.6,6.1,5,4.7,5.6,6,4.6,7.9,7,7.1,6.1,5.6,6.1,12.9,12.8,12.2,13.7,7.1,4.4,5.9,10.9,12.8,12.5,17.4,9.2,0,2,3.3,2.9,3,3.9,11.6,18.3,14.6,12.9,12.5,11.6},//Shamelin
		{4.3,4.6,5.4,5.4,5.7,4.6,4.4,5.3,5.6,4.3,7.6,6.7,6.8,5.8,5.3,7.1,8.9,10.2,8.7,11.1,6.8,4.1,4.1,9.9,13,11.3,15,10.3,1.8,0,2.3,1.9,1.7,2.6,11.7,18.3,14.7,14.3,14.8,12.9},//ATMSC
		{22.9,23.3,23.6,22.6,23.8,22,22.7,24.9,23.5,22.7,25.6,25,25.1,24.1,23.9,26.5,24.9,26.1,24.7,28.2,25.2,22.2,22.1,15.3,27.4,23.8,15.3,21.5,21.5,22.4,0,0.07,21.7,23.3,26.2,27.5,28.3,27.4,15.1,15.6},// IKEA
		{2.1,2.5,3.3,2.2,3.4,2.5,2.3,3.1,3.1,2.1,5.3,4.4,4.5,3.5,3.2,5.3,7.7,9.5,5.3,8.3,4.6,1.8,2.2,7.8,11.2,11.7,15.5,7.6,3.3,2.4,0.23,0,1.7,2.3,10,16.9,13,12.5,15.3,10.6},//Mytown
		{3.1,3.5,4.2,4,4.3,2.8,4.1,4.1,3.8,3.1,6.2,5.2,5.2,4.4,5,8,8.1,9.3,7.9,9.2,5.4,2.4,2.8,8,11.5,9.8,15.8,7.9,3.3,1.3,1.1,0.65,0,1.6,10.3,15.9,13.3,11.6,15.5,10.8},//SunwayVM
		{6.2,6.6,7.4,6.3,7.7,4.5,6.4,7.2,7.4,6.2,9.6,8.7,8.8,7.8,8.7,7.8,8.7,10,8.5,11.5,9,4.1,5.9,9.6,12.8,11.1,15,10,4.4,2.2,5.3,4.9,2.1,0,11.5,17,14.5,14.1,14.8,13.1},//viva
		{9,9.3,9.6,8.7,10.3,9.4,8.8,8,8.4,8.8,9.4,10.5,9.3,10.9,7.2,17.1,6.4,7.7,6.2,7.7,6.7,11.2,9.8,12,1.5,3.3,14.6,6.4,13.8,11.5,10.6,11.9,11.4,10.2,0,5.8,6.3,4.6,10.7,9.8},//DCM
		{13.7,14,14.3,13.4,13.4,14,13.5,12.7,13.1,13.5,14,15.2,14,15.6,11.8,21.8,11.1,12.4,10.9,12.4,11.4,16.3,16.2,13.8,6.6,7.9,20.8,10.5,18.5,16.2,15.3,16,16.1,14.8,5.7,0,7.6,9.3,12.6,11.7},//GloDamansara
		{11.5,11.8,9.2,11.2,12.1,11.9,11.3,8.8,11.8,11.3,8.4,9.1,8.3,9.5,8.1,12.8,7.1,6.7,7.9,6.3,8.4,13.7,11.5,14.6,7.4,10.8,23.7,8.9,16.3,14,12.8,12.6,13.9,11.8,4.9,7.7,0,1.7,13.3,12.4},//HSC
		{10.1,10.4,7.8,9.8,8,10.4,9.9,7.4,7.7,9.9,7,7.7,6.9,8.1,6.7,11.4,5.7,5.3,6.5,4.9,7,8.7,10.1,14.5,6.3,8.5,22.3,7.4,14.9,12.6,11.4,11.2,12.5,10.4,5,10.6,2,0,14.8,13.9},//Publika
		{10.9,11.2,11.5,10.6,11.9,11.2,10.7,11.8,15.1,10.7,13.5,15.2,15.2,14.2,10.5,19.9,10.5,11.8,10.3,14.2,10.8,14,11.1,7.9,9.8,8.1,4.3,6.7,14.9,11.3,13.8,13.6,11.2,11.5,12.1,12.7,13.5,13.4,0,1},//PlzaOUG
		{10.4,10.7,11.1,10.2,11.8,10.8,10.2,11.4,11.5,10.2,13,14.7,14.8,13.8,10.1,16.5,10.1,11.4,9.9,13.8,10.4,10.2,10.7,5.8,8.7,7,5.5,6.3,13.2,10.9,15,14.8,10.8,11,11.7,12.2,13.1,13,1.4,0},//pearlpoint


};

// minDistance for distanceMatrix
int minDistance(double dist[], bool sptSet[]) {
	double min = DBL_MAX;
	int min_index;

	for (int v = 0; v < V; v++) {
		if (!sptSet[v] && dist[v] < min) {
			min = dist[v];
			min_index = v;
		}
	}

	return min_index;
}


// Function to print the path from source to target
void printPath(int parent[], int target) {
	if (parent[target] == -1)
		return;
	printPath(parent, parent[target]);
	int targetNum = target + 1;
	printf("-> %s (%d) ", Malls[target], targetNum);
}

void dijkstraTime(double timeGraph[V][V], int src, int target) {
	int visited[V]; // Keep track of visited nodes
	double dist[V]; // Distance array to store the shortest distances
	int parent[V]; // Parent array to store the shortest path

	for (int i = 0; i < V; i++) {
		visited[i] = 0;    // Mark all vertices as not visited
		dist[i] = INT_MAX; // Initialize distances to infinity
		parent[i] = -1;    // Initialize parents to -1 (undefined)
	}

	dist[src] = 0;// Distance from source to source is 0

	for (int count = 0; count < V - 1; count++) {
		// Find the vertex with the minimum distance value
		int u = -1;
		for (int v = 0; v < V; v++) {
			if (!visited[v] && (u == -1 || dist[v] < dist[u])) {
				u = v;
			}
		}

		visited[u] = 1; // Mark the selected vertex as visited

		// Update dist value of the adjacent vertices
		for (int v = 0; v < V; v++) {
			if (!visited[v] && travellingTimeMatrix[u][v] && dist[u] != INT_MAX &&
				dist[u] + travellingTimeMatrix[u][v] < dist[v]) {
				dist[v] = dist[u] + travellingTimeMatrix[u][v];
				parent[v] = u;
			}
		}
	}

	// Print the shortest path from src to target
	int srcNum = src + 1;
	int targetNum = target + 1;
	printf("Shortest Travelling Time from %s (%d) to %s (%d): ", Malls[src], srcNum, Malls[target], targetNum);
	// Convert total time in seconds to hours, minutes, and seconds
	int hours = (int)dist[target];
	double remainder = dist[target] - hours;
	int minutes = (int)(remainder * 60);
	remainder = remainder * 60 - minutes;
	int seconds = (int)(remainder * 60);

	printf("%02d hrs:%02d min:%02d sec\n", hours, minutes, seconds);

}


// Function to calculate and print the shortest path using Dijkstra's algorithm
void dijkstraPath(double graph[V][V], int src, int target, char* Malls[]) {
	double dist[V];
	bool sptSet[V];
	int parent[V];

	for (int i = 0; i < V; i++) {
		dist[i] = INT_MAX;
		sptSet[i] = false;
	}

	dist[src] = 0;
	parent[src] = -1;
	int srcNum = src + 1;
	int targetNum = target + 1;

	for (int count = 0; count < V - 1; count++) {
		int u = minDistance(dist, sptSet);
		sptSet[u] = true;

		for (int v = 0; v < V; v++) {
			if (!sptSet[v] && graph[u][v] && dist[u] != INT_MAX && dist[u] + graph[u][v] < dist[v]) {
				dist[v] = dist[u] + graph[u][v];
				parent[v] = u;
			}
		}
	}

	if (dist[target] == DBL_MAX) {
		printf("No path from %s to %s\n", Malls[src], Malls[target]);
	}
	else {
		printf("Shortest path from %s (%d) to %s (%d): %s (%d)", Malls[src], srcNum, Malls[target], targetNum, Malls[src], srcNum);
		printPath(parent, target);
		printf("\nShortest distance: %.2fKm\n", dist[target]);
		shortestDistanceMatrix[src][target] = dist[target];
	}
}


static int pathSpeeds[V][V]; //to keep the speeds generated for a particular route
static int pathUserCount[V][V]; //the number of users who have requested the same route



int generateRandomSpeed() {
	int const MAX_SPEED = 120;
	int const MIN_SPEED = 30;
	return MIN_SPEED + (rand() % (MAX_SPEED - MIN_SPEED + 1));
}


void generateUserQuery(void* pvParameters) {
	Queries* queries = (Queries*)pvParameters;
	srand(time(0));
	const TickType_t xDelay250ms = pdMS_TO_TICKS(0);
	User user;

	int numUsers = 1;

	for (numUsers = 1; numUsers <= queries->noQueries; numUsers++) {
		user.startTime = xTaskGetTickCount();
		do {
			user.src = rand() % V;
			user.userID = numUsers;

			user.speed = generateRandomSpeed();
			user.destination = rand() % V;
		} while (user.src == user.destination);

		const char* srcMallName = Malls[user.src];
		const char* destMallName = Malls[user.destination];
		int srcNum = user.src + 1;
		int destNum = user.destination + 1;

		pathSpeeds[user.src][user.destination] = pathSpeeds[user.src][user.destination] + user.speed;
		pathUserCount[user.src][user.destination]++;
		
		printf("\nUser Query: %d\n", user.userID);
		printf("Source to Destination: %s (%d) -> %s (%d)\n", srcMallName, srcNum, destMallName, destNum);
		printf("Speed: %dkm/h\n", user.speed);

		
		xQueueSend(userQueue, &user, portMAX_DELAY);//send user to processQuery

		vTaskDelay(xDelay250ms);
	}
	xSemaphoreGive(queueSemaphore);

}

void processQuery(void* pvParameters) {
	User user;
	Response resp;
	unsigned long globalMinElapsedTime = ULONG_MAX;  // Initialize globalMinElapsedTime to maximum possible value
	unsigned long globalMaxElapsedTime = 0;          // Initialize globalMaxElapsedTime to 0
	unsigned long totalElapsedTime = 0;
	unsigned long numQueries = 0;

	while (1) {

		if (xQueueReceive(userQueue, &user, portMAX_DELAY) == pdPASS) {
			dijkstraPath(distanceMatrix, user.src, user.destination, Malls);

			
			if (pathUserCount[user.src][user.destination] > 0) {//if a path was taken by atleast a single user
				double averageSpeed = (double)pathSpeeds[user.src][user.destination] / pathUserCount[user.src][user.destination]; //get average speed
				
				//distance equals to shortest distance
				double distance = shortestDistanceMatrix[user.src][user.destination];

				//update travelling time matrix
				travellingTimeMatrix[user.src][user.destination] = distance / averageSpeed;
			}
			//get shortest travellling time
			dijkstraTime(travellingTimeMatrix, user.src, user.destination);

			TickType_t endTime = xTaskGetTickCount();

			user.elapsedTime = endTime - user.startTime;

			// Update globalMinElapsedTime and globalMaxElapsedTime
			if (user.elapsedTime < globalMinElapsedTime) {
				globalMinElapsedTime = user.elapsedTime;
			}
			if (user.elapsedTime > globalMaxElapsedTime) {
				globalMaxElapsedTime = user.elapsedTime;
			}

			// Accumulate total elapsed time and increment the number of queries
			totalElapsedTime += user.elapsedTime;
			numQueries++;

			//display elapsed time
			printf("Elapsed Time: %lu milliseconds\n", user.elapsedTime);
			printf("-----------------------------------------------------------------------------------------\n");
			printf("-----------------------------------------------------------------------------------------\n");

			
			

		}
		if (numQueries > 0) {
			double averageElapsedTime = (double)totalElapsedTime / numQueries;
			resp.avgElapsedTime = averageElapsedTime;
		}

		
		resp.maxElapsedTime = globalMaxElapsedTime;
		resp.minElapsedTime = globalMinElapsedTime;
		resp.totalElapsedTime = totalElapsedTime;


		//send to response task
		xQueueSend(responseQueue, &resp, portMAX_DELAY);
		

	}
}

//Task to display the min,max,total and average elapsed time
void response(void* pvParameters) {
	Response resp;

	while (1) {
		if (xQueueReceive(responseQueue, &resp, portMAX_DELAY) == pdPASS) {
			if (xSemaphoreTake(queueSemaphore, portMAX_DELAY == pdPASS)) {
				printf("\n\n***************************************************************************\n");
				printf("\t\tRUNNING TIME\n");
				printf("***************************************************************************\n");
				printf("\t%-25s: %lu milliseconds\n", "Minimum Elapsed Time", resp.minElapsedTime);
				printf("\t%-25s: %lu milliseconds\n", "Maximum Elapsed Time", resp.maxElapsedTime);
				printf("\t%-25s: %lu milliseconds\n", "Total Elapsed Time", resp.totalElapsedTime);
				printf("\t%-25s: %.3f milliseconds\n", "Average Elapsed Time", resp.avgElapsedTime);
				printf("***************************************************************************\n");

			}
			

		}


	}
}


int main() {
	
	userQueue = xQueueCreate(10, sizeof(User));
	responseQueue = xQueueCreate(10, sizeof(Response));
	queueSemaphore = xSemaphoreCreateBinary();
	Queries queries;

	//user menu
	int numQueries;
	printf("Enter how many queries you would like to generate: ");
	scanf("%d", &numQueries);
	queries.noQueries = numQueries;


	//tasks
	xTaskCreate(generateRandomSpeed, "RandSpeed", 1024, NULL, 1, NULL);
	xTaskCreate(generateUserQuery, "UserQ", 1024, &queries, 1, NULL);
	xTaskCreate(processQuery, "ProcessQuery", 1024, NULL, 1, NULL);
	xTaskCreate(response, "Response", 1024, NULL, 1, NULL);
	
	vTaskStartScheduler();


	return 0;

}














