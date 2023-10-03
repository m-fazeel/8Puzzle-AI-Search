# AI-8Puzzle-Solver

This project is devised to tackle the Expense 8 Puzzle Problem utilizing uninformed and informed search strategies. Implemented in Python, it demonstrates a variety of search algorithms alongside an in-depth analysis of each solution path.

## Table of Contents
- [AI-8Puzzle-Solver](#ai-8puzzle-solver)
  - [Table of Contents](#table-of-contents)
  - [Introduction](#introduction)
  - [Input Specification](#input-specification)
  - [Search Strategies](#search-strategies)
  - [Code Structure](#code-structure)
  - [Heuristic Function](#heuristic-function)
  - [Overview of the Search Algorithms](#overview-of-the-search-algorithms)
  - [Running the Code](#running-the-code)
  - [Copyright](#copyright)

## Introduction
The Expense 8 Puzzle is a classic problem in AI. This project explores different search strategies to find the optimal solution path from a given start state to a defined goal state.

## Input Specification
The agent accepts command line arguments specifying the start state, goal state, and search method.

## Search Strategies
The following search strategies have been implemented:
- Breadth-First Search (BFS)
- Uniform Cost Search (UCS)
- Depth-First Search (DFS)
- Depth-Limited Search (DLS)
- Iterative Deepening Search (IDS)
- Greedy Best-First Search (Greedy)
- A* Search

A suitable heuristic is designed for the Greedy and A* Search methods to optimize the search.

## Code Structure
- **Class Node**: Contains the current node, parent node, action, depth, cost, heuristic value, and the action taken to reach the state.
    - Each search algorithm has its own function.
    - Common functions like `blank_tile`, `actions_to_perform`, `move_tile`, `expand_node`, and file writing functions are utilized by each search algorithm.

## Heuristic Function
- **Heuristic function (def h)**: Calculates the heuristic value based on a modified Manhattan distance, which is the sum of the product of the Manhattan distance and the number on the tile.

## Overview of the Search Algorithms
Detailed explanations of how each algorithm operates are provided, including the data structures and mechanisms used for node storage, expansion, and goal checking.

## Running the Code
To execute the code, use the following command in a terminal with Python installed (ensure it's in the same directory as the code):
```bash
python3 expense_8_puzzle.py <input_file> <output_file> <search_algorithm> <dump_flag>
```
- `input_file`: Input file containing the initial state of the puzzle.
output_file: Output file for storing the final state of the puzzle.

- `search_algorithm`: Algorithm used to solve the puzzle.
- `dump_flag`: Optional. Enter true or True if a dump file is needed; otherwise, leave it blank or enter false. If no method is specified, A* is used by default.

For example, to run A* search, parse the command as follows:
```
python3 expense_8_puzzle.py <input_file> <output_file> a_star <dump_flag>
```

## Copyright
This software is provided for educational purposes only. It is prohibited to use this code, for any college assignment or personal use. Unauthorized distribution, modification or commercial usage is strictly forbidden. Please respect the rights of the author.
