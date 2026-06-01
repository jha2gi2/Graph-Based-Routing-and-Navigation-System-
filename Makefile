# Compiler to use
CXX = g++

# --- General Flags ---
# -std=c++17: Use the C++17 standard
# -g: Include debug symbols
# -Wall -Wextra: Enable all major warnings
BASE_CXXFLAGS = -std=c++17 -O3 -march=native

# --- Phase 1 ---
TARGET_P1 = phase1
CXXFLAGS_P1 = $(BASE_CXXFLAGS) -IPhase\ 1
SRC_P1 = Phase\ 1/SampleDriver.cpp Phase\ 1/Graph.cpp Phase\ 1/QueryProcessor.cpp Phase\ 1/ShortestPath.cpp Phase\ 1/knn.cpp
HEADERS_P1 = Phase\ 1/Graph.hpp Phase\ 1/QueryProcessor.hpp Phase\ 1/ShortestPath.hpp Phase\ 1/knn.hpp Phase\ 1/json.hpp

# --- Phase 2 ---
TARGET_P2 = phase2
CXXFLAGS_P2 = $(BASE_CXXFLAGS) -IPhase\ 2
SRC_P2 = Phase\ 2/SampleDriver.cpp Phase\ 2/Graph.cpp Phase\ 2/QueryProcessor.cpp Phase\ 2/k_shortest_paths.cpp Phase\ 2/k_shortest_paths_heuristic.cpp Phase\ 2/approx_shortest_paths.cpp
HEADERS_P2 = Phase\ 2/Graph.hpp Phase\ 2/QueryProcessor.hpp Phase\ 2/k_shortest_paths.hpp Phase\ 2/k_shortest_paths_heuristic.hpp Phase\ 2/approx_shortest_paths.hpp Phase\ 2/json.hpp

# --- Phase 3 ---
TARGET_P3 = phase3
# Include path for Phase 3 headers
CXXFLAGS_P3 = $(BASE_CXXFLAGS) -IPhase\ 3

# Source files
SRC_P3 = Phase\ 3/SampleDriver.cpp Phase\ 3/Graph.cpp Phase\ 3/QueryProcessor.cpp Phase\ 3/tsp.cpp

# Header files
HEADERS_P3 = Phase\ 3/Graph.hpp Phase\ 3/QueryProcessor.hpp Phase\ 3/tsp.hpp Phase\ 3/json.hpp

# --- Build Rules ---

# The 'all' target builds all phases
all: $(TARGET_P1) $(TARGET_P2) $(TARGET_P3)

# Rule to build the 'phase1' executable
$(TARGET_P1): $(SRC_P1) $(HEADERS_P1)
	$(CXX) $(CXXFLAGS_P1) -o $(TARGET_P1) $(SRC_P1)

# Rule to build the 'phase2' executable
$(TARGET_P2): $(SRC_P2) $(HEADERS_P2)
	$(CXX) $(CXXFLAGS_P2) -o $(TARGET_P2) $(SRC_P2)

# Rule to build the 'phase3' executable
$(TARGET_P3): $(SRC_P3) $(HEADERS_P3)
	$(CXX) $(CXXFLAGS_P3) -o $(TARGET_P3) $(SRC_P3)

# --- Clean Target ---
clean:
	@echo "Cleaning up compiled binaries..."
	rm -f $(TARGET_P1) $(TARGET_P2) $(TARGET_P3)

# Phony targets
.PHONY: all clean