#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <limits.h>

#define MAX_VERTICES 100

typedef struct Edge {
    int source;
    int destination;
    int weight;
} Edge;

typedef struct Graph {
    int numVertices;
    int numEdges;
    Edge* edges[MAX_VERTICES];
} Graph;

typedef struct MinHeapNode {
    int vertex;
    int distance;
} MinHeapNode;

typedef struct MinHeap {
    int size;
    int capacity;
    int* positions;
    MinHeapNode** nodes;
} MinHeap;

Graph* createGraph(int numVertices, int numEdges) {
    Graph* graph = (Graph*)malloc(sizeof(Graph));
    graph->numVertices = numVertices;
    graph->numEdges = numEdges;

    for (int i = 0; i < numVertices; i++) {
        graph->edges[i] = NULL;
    }

    return graph;
}

void addEdge(Graph* graph, int source, int destination, int weight) {
    Edge* edge = (Edge*)malloc(sizeof(Edge));
    edge->source = source;
    edge->destination = destination;
    edge->weight = weight;

    graph->edges[source] = edge;
}

MinHeapNode* createMinHeapNode(int vertex, int distance) {
    MinHeapNode* node = (MinHeapNode*)malloc(sizeof(MinHeapNode));
    node->vertex = vertex;
    node->distance = distance;
    return node;
}

MinHeap* createMinHeap(int capacity) {
    MinHeap* minHeap = (MinHeap*)malloc(sizeof(MinHeap));
    minHeap->size = 0;
    minHeap->capacity = capacity;
    minHeap->positions = (int*)malloc(capacity * sizeof(int));
    minHeap->nodes = (MinHeapNode**)malloc(capacity * sizeof(MinHeapNode*));
    return minHeap;
}

void swapMinHeapNodes(MinHeapNode** a, MinHeapNode** b) {
    MinHeapNode* temp = *a;
    *a = *b;
    *b = temp;
}

void minHeapify(MinHeap* minHeap, int idx) {
    int smallest, left, right;
    smallest = idx;
    left = 2 * idx + 1;
    right = 2 * idx + 2;

    if (left < minHeap->size &&
        minHeap->nodes[left]->distance < minHeap->nodes[smallest]->distance) {
        smallest = left;
    }

    if (right < minHeap->size &&
        minHeap->nodes[right]->distance < minHeap->nodes[smallest]->distance) {
        smallest = right;
    }

    if (smallest != idx) {
        MinHeapNode* smallestNode = minHeap->nodes[smallest];
        MinHeapNode* idxNode = minHeap->nodes[idx];

        minHeap->positions[smallestNode->vertex] = idx;
        minHeap->positions[idxNode->vertex] = smallest;

        swapMinHeapNodes(&minHeap->nodes[smallest], &minHeap->nodes[idx]);

        minHeapify(minHeap, smallest);
    }
}

int isEmpty(MinHeap* minHeap) {
    return minHeap->size == 0;
}

MinHeapNode* extractMin(MinHeap* minHeap) {
    if (isEmpty(minHeap)) {
        return NULL;
    }

    MinHeapNode* rootNode = minHeap->nodes[0];
    MinHeapNode* lastNode = minHeap->nodes[minHeap->size - 1];

    minHeap->nodes[0] = lastNode;

    minHeap->positions[rootNode->vertex] = minHeap->size - 1;
    minHeap->positions[lastNode->vertex] = 0;

    --minHeap->size;
    minHeapify(minHeap, 0);

    return rootNode;
}

void decreaseKey(MinHeap* minHeap, int vertex, int distance) {
    int i = minHeap->positions[vertex];
    minHeap->nodes[i]->distance = distance;

    while (i && minHeap->nodes[i]->distance < minHeap->nodes[(i - 1) / 2]->distance) {
        minHeap->positions[minHeap->nodes[i]->vertex] = (i - 1) / 2;
        minHeap->positions[minHeap->nodes[(i - 1) / 2]->vertex] = i;
        swapMinHeapNodes(&minHeap->nodes[i], &minHeap->nodes[(i - 1) / 2]);

        i = (i - 1) / 2;
    }
}

void printDijkstraResult(int* distances, int numVertices, int source) {
    printf("Dijkstra's Algorithm\n");
    printf("Vertex\tDistance from Source\n");
    for (int i = 0; i < numVertices; i++) {
        printf("%d\t%d\n", i, distances[i]);
    }
    printf("\n");
}

void dijkstra(Graph* graph, int source) {
    int numVertices = graph->numVertices;
    int* distances = (int*)malloc(numVertices * sizeof(int));

    MinHeap* minHeap = createMinHeap(numVertices);

    for (int v = 0; v < numVertices; v++) {
        distances[v] = INT_MAX;
        minHeap->nodes[v] = createMinHeapNode(v, distances[v]);
        minHeap->positions[v] = v;
    }

    minHeap->size = numVertices;

    distances[source] = 0;
    decreaseKey(minHeap, source, distances[source]);

    while (!isEmpty(minHeap)) {
        MinHeapNode* minHeapNode = extractMin(minHeap);
        int u = minHeapNode->vertex;

        Edge* edge = graph->edges[u];
        while (edge != NULL) {
            int v = edge->destination;
            if (distances[u] != INT_MAX && edge->weight + distances[u] < distances[v]) {
                distances[v] = distances[u] + edge->weight;
                decreaseKey(minHeap, v, distances[v]);
            }
            edge = edge->next;
        }
    }

    printDijkstraResult(distances, numVertices, source);

    free(distances);
}

void DFSUtil(Graph* graph, int vertex, bool visited[]) {
    visited[vertex] = true;
    printf("%d ", vertex);

    Edge* edge = graph->edges[vertex];
    while (edge != NULL) {
        int adjacentVertex = edge->destination;
        if (!visited[adjacentVertex]) {
            DFSUtil(graph, adjacentVertex, visited);
        }
        edge = edge->next;
    }
}

void DFS(Graph* graph, int startVertex) {
    int numVertices = graph->numVertices;
    bool* visited = (bool*)malloc(numVertices * sizeof(bool));

    for (int i = 0; i < numVertices; i++) {
        visited[i] = false;
    }

    printf("Depth-First Search (DFS): ");
    DFSUtil(graph, startVertex, visited);
    printf("\n");

    free(visited);
}

void BFS(Graph* graph, int startVertex) {
    int numVertices = graph->numVertices;
    bool* visited = (bool*)malloc(numVertices * sizeof(bool));

    for (int i = 0; i < numVertices; i++) {
        visited[i] = false;
    }

    visited[startVertex] = true;

    int queue[MAX_VERTICES];
    int front = 0, rear = 0;
    queue[rear++] = startVertex;

    printf("Breadth-First Search (BFS): ");

    while (front < rear) {
        int vertex = queue[front++];
        printf("%d ", vertex);

        Edge* edge = graph->edges[vertex];
        while (edge != NULL) {
            int adjacentVertex = edge->destination;
            if (!visited[adjacentVertex]) {
                visited[adjacentVertex] = true;
                queue[rear++] = adjacentVertex;
            }
            edge = edge->next;
        }
    }

    printf("\n");

    free(visited);
}

int main() {
    int numVertices, numEdges;
    int sourceVertex;
    int i, source, destination, weight;

    printf("Enter the number of vertices: ");
    scanf("%d", &numVertices);

    printf("Enter the number of edges: ");
    scanf("%d", &numEdges);

    Graph* graph = createGraph(numVertices, numEdges);

    for (i = 0; i < numEdges; i++) {
        printf("\nEnter the details of edge %d:\n", i + 1);
        printf("Source: ");
        scanf("%d", &source);
        printf("Destination: ");
        scanf("%d", &destination);
        printf("Weight: ");
        scanf("%d", &weight);

        addEdge(graph, source, destination, weight);
    }

    printf("\nEnter the source vertex for Dijkstra's algorithm: ");
    scanf("%d", &sourceVertex);

    dijkstra(graph, sourceVertex);
    printf("\n");

    DFS(graph, 0);
    printf("\n");

    BFS(graph, 0);
    printf("\n");

    return 0;
}
