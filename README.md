# Rubik's Cube Solver C++

A C++ program that finds a sequence of face turns to transform an initial Rubik’s Cube configuration into a final configuration.
It implements two search strategies:

* **Iterative Deepening Depth-First Search (IDDFS)** — uninformed.
* **A\*** search with a simple heuristic (number of mismatched stickers) — informed.

This repository contains a straightforward, self-contained solver that models stickers and face turns exactly, accepts 54-sticker cube representations as input, and prints a move sequence (using `F`, `F'`, `B`, `B'`, `U`, `U'`, `D`, `D'`, `L`, `L'`, `R`, `R'` notation).

---

## How it works

### State representation & moves

A cube state is represented as **54 integers** (values `1..6`) representing the colours of the 6 faces:

* **Face order in input:** `Front, Back, Top, Bottom, Left, Right`.
* Each face contains 9 stickers ordered **left→right, top→bottom**.

The program builds a 3D sticker coordinate model and computes the permutation produced by a 90° face rotation. That permutation is used to generate successor states for search.

---

## IDDFS (Iterative Deepening)

Repeated depth-limited DFS from depth `0..limit`.

* Prunes immediate inverse moves (avoid undoing the last move immediately).
* Good for very shallow solutions; memory-efficient.

---

## A\* Search

* **Heuristic:** number of mismatched stickers compared to the goal.
* Maintains an open priority queue (by `f = g + h`) and a closed/visited mapping.
* May expand fewer states than IDDFS for deeper solutions, but performance depends heavily on heuristic quality.

---

## Input format

A text file with **exactly two lines**.

* **Line 1:** initial configuration — 54 space-separated digits (`1..6`).
* **Line 2:** final configuration — 54 space-separated digits (`1..6`).

Each face is 9 digits (left→right, top→bottom). Full order: **Front (9) Back (9) Top (9) Bottom (9) Left (9) Right (9)**.
