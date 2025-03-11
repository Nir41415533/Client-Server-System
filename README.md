# Shortest Path Finder

## Description

This project implements a **client-server** architecture to compute the **shortest path** between two nodes in a graph using **BFS (Breadth-First Search)**.

- The **server** loads a graph from a file and handles client requests.
- The **client** sends source and destination nodes to the server, which responds with the shortest path.

## Features

- Multi-threaded server using **pthreads**.
- Request **caching** to optimize repeated queries.
- Uses **sockets** for communication between client and server.

## Files

- **server.cpp** - The server implementation.
- **client.cpp** - The client implementation.

## Requirements

Ensure you have the following installed:

- **g++** (C++ compiler)
- Linux system with **pthread** and **socket** support

## Compilation

To compile the server and client, run:

```bash
# Compile the server
g++ -o server server.cpp -pthread

# Compile the client
g++ -o client client.cpp
```

## Running the Program

### Start the server:

```bash
./server <graph_file> <port>
```

Example:

```bash
./server graph.txt 8080
```

### Start the client:

```bash
./client <IP> <port> <source> <destination>
```

Example:

```bash
./client 127.0.0.1 8080 1 5
```

## How It Works

1. The **server** reads a graph from a file and starts listening for client connections.
2. The **client** connects to the server and sends two nodes.
3. The server computes the **shortest path** and returns it to the client.
4. The client prints the received result.

## Example Graph File Format

Each line represents an edge between two nodes:

```
1 2
2 3
3 4
4 5
```

## Error Handling

- If no path exists, the server will respond with:
  ```
  No path exists between <source> and <destination>
  ```
- If the input format is incorrect, an error message will be displayed.


## This project is for educational purposes only.

