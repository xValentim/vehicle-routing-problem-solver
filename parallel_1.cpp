#include <iostream>
#include <vector>
#include <fstream>
#include <set>
#include <map>
#include <queue>
#include <utility> // Para std::pair
#include <algorithm>  // Para std::next_permutation
#include <cmath>      // Para pow
#include <climits>
#include <omp.h>
#include <chrono>  // Inclui a biblioteca chrono

using namespace std::chrono;  // Usa o namespace chrono para facilitar

// Define o tipo Edge como uma tupla de três inteiros (origem, destino, custo)
using Edge = std::tuple<int, int, int>;

// Define um grafo como um mapa de inteiros para conjuntos de inteiros
using Graph = std::map<int, std::set<int>>;

// Define um caminho como um vetor de inteiros
using Path = std::vector<int>;

// Define uma demanda como um mapa de inteiros para inteiros
using Demand = std::map<int, int>;

// Define arestas como um mapa de pares de inteiros para inteiros
using Edges = std::map<std::pair<int, int>, int>;

Graph to_matrix_adj(const std::vector<Edge>& edges) {
    Graph graph;
    for (const auto& edge : edges) {
        int origem = std::get<0>(edge);
        int destino = std::get<1>(edge);
        if (origem == 0) origem = -1;   // -1 para 'source'
        if (destino == 0) destino = -2; // -2 para 'sink'
        graph[origem].insert(destino);
    }
    return graph;
}

std::vector<Path> find_all_paths(const Graph& graph) {
    std::vector<Path> paths;
    std::queue<Path> queue;
    Path start = {-1}; // Começa com 'source' representado por -1
    queue.push(start);

    while (!queue.empty()) {
        Path path = queue.front();
        queue.pop();
        int node = path.back();

        if (node == -2) { // -2 para 'sink'
            paths.push_back(path);
        } else {
            for (int neighbor : graph.at(node)) {
                Path new_path(path);
                new_path.push_back(neighbor);
                queue.push(new_path);
            }
        }
    }
    return paths;
}

bool check_capacity(const Path& path, const Demand& demanda, int capacidade) {
    int carga_total = 0;
    for (int node : path) {
        if (node != -1 && node != -2) { // Ignora 'source' e 'sink'
            carga_total += demanda.at(node);
        }
    }
    return carga_total <= capacidade;
}

int calculate_cost(const Path& path, const Edges& arestas) {
    int custo_total = 0;
    
    for (size_t i = 0; i < path.size() - 1; ++i) {
        int origem = path[i];
        if (origem == -1){
            origem = 0;
        }
        
        int destino = path[i + 1];
        if (destino == -2){
            destino = 0;
        }
        
        custo_total += arestas.at({origem, destino});
    }
    return custo_total;
}


int total_cost_of_combination(const std::vector<Path>& combination, const Edges& edges) {
    int total_cost = 0;

    for (size_t j = 0; j < combination.size(); ++j) {
        total_cost += calculate_cost(combination[j], edges);
    }

    return total_cost;
}

std::vector<std::vector<Path>> generate_power_set(const std::vector<Path>& all_paths) {
    std::vector<std::vector<Path>> power_set;
    size_t set_size = all_paths.size();
    size_t power_set_size = std::pow(2, set_size);

    for (size_t counter = 0; counter < power_set_size; counter++) {
        std::vector<Path> subset;
        for (size_t i = 0; i < set_size; i++) {
            if (counter & (1 << i)) {
                subset.push_back(all_paths[i]);
            }
        }
        if (!subset.empty()) {
            power_set.push_back(subset);
        }
    }
    return power_set;
}

bool check_combination_demand(const std::vector<Path>& combination, const Demand& demanda, int max_stops) {
    std::vector<int> demand_nodes;
    Demand demand_copy = demanda;
    for (const auto& [node, demand] : demanda) {
        if (demand > 0) {
            demand_nodes.push_back(node);
        }
    }

    for (const auto& path : combination) {
        if (path.size() - 2 > max_stops) {
            return false;
        }
        for (int node : path) {
            
            if (node != -1 && node != -2) { // Ignora 'source' e 'sink'
                auto it = std::find(demand_nodes.begin(), demand_nodes.end(), node);
                if (it != demand_nodes.end()) {
                    demand_copy[node] = -3;
                }
            }
        }
    }

    for (const auto& [node, demand] : demand_copy) {
        if (demand != -3) {
            return false;
        }
    }

    return true;
}


std::vector<Path> find_best_route_parallel(const std::vector<std::vector<Path>>& power_set, const Edges& edges, const Demand& demanda, int max_stops) {
    int min_cost = INT_MAX;  // Use o valor máximo inicial para min_cost
    std::vector<Path> best_route;
    std::vector<Path> local_best_route;
    int local_min_cost;

    #pragma omp parallel private(local_best_route, local_min_cost)
    {
        local_min_cost = INT_MAX; // Inicialize local_min_cost para o máximo para cada thread

        #pragma omp for nowait
        for (size_t i = 0; i < power_set.size(); ++i) {
            const auto& combination = power_set[i];
            if (check_combination_demand(combination, demanda, max_stops)) {
                int cost = total_cost_of_combination(combination, edges);
                if (cost < local_min_cost) {
                    local_min_cost = cost;
                    local_best_route = combination;
                }
            }
        }

        // Redução para encontrar a rota global com menor custo
        #pragma omp critical
        {
            if (local_min_cost < min_cost) {
                min_cost = local_min_cost;
                best_route = local_best_route;
            }
        }
    }

    return best_route;
}


int main(int argc, char* argv[]) {

    int max_stops = 10;  // Número máximo de paradas permitidas

    if (argc < 2) {  // Verifica se o caminho do arquivo foi fornecido
        std::cerr << "Uso: " << argv[0] << " <caminho_para_o_arquivo>" << std::endl;
        return 1;
    }

    const char* file_path = argv[1];  // Pega o caminho do arquivo do primeiro argumento
    std::ifstream file(file_path);  // Abre o arquivo


    //std::ifstream file("./data/grafo_10.txt");  // Ajuste o caminho conforme necessário
    if (!file) {
        std::cerr << "Não foi possível abrir o arquivo." << std::endl;
        return 1;
    }

    std::cout << "Arquivo aberto com sucesso.\n" << "File path: " << file_path << "\n";
    std::cout << "Parallel 1 (OpenMP)" << "\n";

    auto start = high_resolution_clock::now();
    int num_nos, num_arestas;
    file >> num_nos;  // Lê o número de nós

    Demand demanda;
    int node_index, node_demand;

    // Ler as demandas dos nós
    for (int i = 0; i < num_nos - 1; ++i) {
        file >> node_index >> node_demand;
        demanda[node_index] = node_demand;
    }

    file >> num_arestas;  // Lê o número de arestas
    std::vector<Edge> edges;
    int origem, destino, custo;

    // Ler as arestas
    for (int i = 0; i < num_arestas; ++i) {
        file >> origem >> destino >> custo;
        edges.push_back(std::make_tuple(origem, destino, custo));
    }

    file.close();

    // Criar o mapa de arestas
    Edges edge_map;
    for (const auto& e : edges) {
        edge_map[{std::get<0>(e), std::get<1>(e)}] = std::get<2>(e);
    }

    // Continuar com a lógica anterior para gerar o grafo, encontrar caminhos, etc.
    Graph graph = to_matrix_adj(edges);

    // Find all paths
    std::vector<Path> all_paths = find_all_paths(graph);

    // Gerar o conjunto das partes de todos os caminhos
    std::vector<std::vector<Path>> power_set_paths = generate_power_set(all_paths);

    // Encontrar a melhor rota que atende à demanda com o menor custo
    std::vector<Path> best_route = find_best_route_parallel(power_set_paths, edge_map, demanda, max_stops);

    // Finaliza o timer e calcula a duração
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);

    // Exibir a melhor rota encontrada
    std::cout << "Melhor rota encontrada:\n";
    for (const auto& path : best_route) {
        for (int node : path) {
            std::cout << node << " -> ";
        }
        std::cout << "Fim\n";
    }

    std::cout << "Tempo total de execução: " << duration.count() << " ms" << std::endl;

    return 0;
}