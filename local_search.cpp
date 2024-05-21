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



std::vector<Path> find_all_paths(const Graph& graph, const Demand& demanda) {
    std::vector<Path> paths;
    std::queue<Path> queue;
    Demand demanda_copy = demanda;
    // std::vector<int> neighbors;
    Path start = {-1}; // Começa com 'source' representado por -1
    queue.push(start);

    while (!queue.empty()) {
        Path path = queue.front();
        queue.pop();
        int node = path.back();

        if (node == -2) { // -2 para 'sink'
            paths.push_back(path);
        } else {

            std::vector<int> neighbors(graph.at(node).begin(), graph.at(node).end());

            if (std::find(neighbors.begin(), neighbors.end(), -2) != neighbors.end()){
                // Removendo o 'sink' do vetor
                neighbors.erase(std::remove(neighbors.begin(), neighbors.end(), -2), neighbors.end());

                // Ordenando os vizinhos baseados na demanda, em ordem decrescente
                std::sort(neighbors.begin(), neighbors.end(), [&demanda_copy](int a, int b) {
                    return demanda_copy[a] > demanda_copy[b];
                });

                // Adicionando 'sink' de volta ao início do vetor
                neighbors.insert(neighbors.begin(), -2);
            } else {
                // Apenas ordena
                std::sort(neighbors.begin(), neighbors.end(), [&demanda_copy](int a, int b) {
                    return demanda_copy[a] > demanda_copy[b];
                });   
            }

            if (node == -1){

                for (int neighbor : neighbors) {
                    Path new_path(path);
                    new_path.push_back(neighbor);
                    queue.push(new_path);
                }

            } else {

                int len = neighbors.size();

                // Definindo os limites do slice
                int start = 0;  // Primeiro elemento
                int end = len;  // Último elemento
                int max_range = 3;  // Máximo de vizinhos a serem considerados

                int range = std::min(max_range, end);

                // Criando o subvector usando iteradores
                std::vector<int> sliced(neighbors.begin() + start, neighbors.begin() + range);

                for (int neighbor : sliced) {
                    Path new_path(path);
                    new_path.push_back(neighbor);
                    queue.push(new_path);
                }
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
    // printa todas as arestas com pesos
    // for (const auto& [aresta, peso] : arestas) {
    //     std::cout << aresta.first << " -> " << aresta.second << " : " << peso << std::endl;
    // }

    for (size_t i = 0; i < path.size() - 1; ++i) {
        int origem = path[i];
        if (origem == -1){
            origem = 0;
        }
        
        int destino = path[i + 1];
        if (destino == -2){
            destino = 0;
        }
        
        // std::cout << origem << std::endl;
        // std::cout << destino << std::endl;
        
        custo_total += arestas.at({origem, destino});
    }
    return custo_total;
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

bool check_combination_demand(const std::vector<Path>& combination, const Demand& demanda) {
    std::vector<int> demand_nodes;
    Demand demand_copy = demanda;
    for (const auto& [node, demand] : demanda) {
        if (demand > 0) {
            demand_nodes.push_back(node);
        }
    }

    // std::cout << "Lista de demanda" << std::endl;
    // for (const auto& node : demand_nodes) {
    //     std::cout << node << " ";
    // }

    for (const auto& path : combination) {
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

int total_cost_of_combination(const std::vector<Path>& combination, const Edges& edges) {
    int total_cost = 0;
    for (const auto& path : combination) {
        total_cost += calculate_cost(path, edges);
        // std::cout << "Entrei no for do total cost e alem" << std::endl;
    }
    return total_cost;
}

std::vector<Path> find_best_route(const std::vector<std::vector<Path>>& power_set, const Edges& edges, const Demand& demanda) {
    int min_cost = 99999999;
    std::vector<Path> best_route;
    for (const auto& combination : power_set) {
        if (check_combination_demand(combination, demanda)) {
            // std::cout << "Entrei no IF " << std::endl;
            int cost = total_cost_of_combination(combination, edges);
            // std::cout << "Custo: " << cost << std::endl;
            if (cost < min_cost) {
                min_cost = cost;
                best_route = combination;
            }
        }
    }
    return best_route;
}

int main() {
    std::ifstream file("./data/grafo_11.txt");  // Ajuste o caminho conforme necessário
    if (!file) {
        std::cerr << "Não foi possível abrir o arquivo." << std::endl;
        return 1;
    }

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
    std::vector<Path> all_paths = find_all_paths(graph, demanda);

    // Mostra all_paths
    // std::cout << "Rota encontrada:\n";
    // for (const auto& path : all_paths) {
    //     for (int node : path) {
    //         std::cout << node << " -> ";
    //     }
    //     std::cout << "Fim\n";
    // }

    // Gerar o conjunto das partes de todos os caminhos
    std::vector<std::vector<Path>> power_set_paths = generate_power_set(all_paths);

    // Mostra power set
    // std::cout << "Conjunto das partes de todos os caminhos:\n";
    // for (const auto& combination : power_set_paths) {
    //     for (const auto& path : combination) {
    //         for (int node : path) {
    //             std::cout << node << " -> ";
    //         }
    //         std::cout << "Fim\n";
    //     }
    //     // std::cout << "Custo total: " << total_cost_of_combination(combination, edge_map) << "\n";
    //     std::cout << "-------------------------" << "\n";
    // }
    
    // Encontrar a melhor rota que atende à demanda com o menor custo
    std::vector<Path> best_route = find_best_route(power_set_paths, edge_map, demanda);

    // Exibir a melhor rota encontrada
    std::cout << "Melhor rota encontrada:\n";
    for (const auto& path : best_route) {
        for (int node : path) {
            std::cout << node << " -> ";
        }
        std::cout << "Fim\n";
    }

    return 0;
}

/*

[(), 
({'path': ['source', 1, 'sink'], 'cost': 64},), 
({'path': ['source', 2, 'sink'], 'cost': 58},), 
({'path': ['source', 3, 'sink'], 'cost': 36},), 
({'path': ['source', 2, 3, 'sink'], 'cost': 59},), 
({'path': ['source', 1, 'sink'], 'cost': 64}, {'path': ['source', 2, 'sink'], 'cost': 58}), 
({'path': ['source', 1, 'sink'], 'cost': 64}, {'path': ['source', 3, 'sink'], 'cost': 36}), 
({'path': ['source', 1, 'sink'], 'cost': 64}, {'path': ['source', 2, 3, 'sink'], 'cost': 59}), 
({'path': ['source', 2, 'sink'], 'cost': 58}, {'path': ['source', 3, 'sink'], 'cost': 36}), 
({'path': ['source', 2, 'sink'], 'cost': 58}, {'path': ['source', 2, 3, 'sink'], 'cost': 59}), 
({'path': ['source', 3, 'sink'], 'cost': 36}, {'path': ['source', 2, 3, 'sink'], 'cost': 59}), 
({'path': ['source', 1, 'sink'], 'cost': 64}, {'path': ['source', 2, 'sink'], 'cost': 58}, {'path': ['source', 3, 'sink'], 'cost': 36}), 
({'path': ['source', 1, 'sink'], 'cost': 64}, {'path': ['source', 2, 'sink'], 'cost': 58}, {'path': ['source', 2, 3, 'sink'], 'cost': 59}), 
({'path': ['source', 1, 'sink'], 'cost': 64}, {'path': ['source', 3, 'sink'], 'cost': 36}, {'path': ['source', 2, 3, 'sink'], 'cost': 59}), 
({'path': ['source', 2, 'sink'], 'cost': 58}, {'path': ['source', 3, 'sink'], 'cost': 36}, {'path': ['source', 2, 3, 'sink'], 'cost': 59}), 
({'path': ['source', 1, 'sink'], 'cost': 64}, {'path': ['source', 2, 'sink'], 'cost': 58}, {'path': ['source', 3, 'sink'], 'cost': 36}, {'path': ['source', 2, 3, 'sink'], 'cost': 59})]


[['source', 1, 'sink'], 
['source', 2, 'sink'], 
['source', 3, 'sink'], 
['source', 4, 'sink'], 
['source', 5, 'sink'], 
['source', 6, 'sink'], 
['source', 7, 'sink'],
['source', 8, 'sink'], 
['source', 9, 'sink'], 
['source', 10, 'sink'], 
['source', 1, 8, 'sink'], 
['source', 1, 7, 'sink'], 
['source', 2, 4, 'sink'], 
['source', 3, 8, 'sink'], 
['source', 3, 9, 'sink'], 
['source', 3, 6, 'sink'], 
['source', 6, 9, 'sink'], 
['source', 9, 10, 'sink'], 
['source', 3, 9, 10, 'sink'], 
['source', 3, 6, 9, 'sink'], 
['source', 6, 9, 10, 'sink'], 
['source', 3, 6, 9, 10, 'sink']]
*/