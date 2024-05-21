# Vehicle Routing Problem (VRP) Solver

## Introdução

O problema de roteamento de veículos (Vehicle Routing Problem, VRP) é uma classe de problemas de otimização combinatória que envolve a determinação das rotas ótimas para uma frota de veículos destinados a entregar produtos a diversos clientes. O objetivo é minimizar o custo total das rotas, que pode incluir distância, tempo, consumo de combustível, entre outros, enquanto se cumpre com as restrições de capacidade dos veículos e a demanda dos clientes.

## Solução de Força Bruta

### Descrição da Solução

A abordagem de força bruta para resolver o VRP implementada neste projeto segue os seguintes passos:

1. **Geração de Rotas Possíveis:** Utiliza-se um algoritmo de busca em largura (Breadth-First Search, BFS) para explorar todas as possíveis rotas que podem ser formadas a partir do ponto de partida. Cada rota é construída incrementalmente, verificando todas as conexões possíveis a partir de cada nó.

2. **Geração do Conjunto de Potência (Power Set):** Após a geração de todas as rotas possíveis, criamos um conjunto de potência dessas rotas. O conjunto de potência contém todos os subconjuntos possíveis de rotas, permitindo a combinação de diferentes rotas para formar soluções abrangentes.

3. **Seleção de Rotas:** Dentro do conjunto de potência, seleciona-se o subconjunto que apresenta o menor custo total, desde que esse subconjunto respeite as restrições de demanda do problema. Isso envolve verificar se a capacidade total transportada em cada rota do subconjunto não excede a demanda requerida pelos clientes.

### Limitações

A solução de força bruta é computacionalmente intensiva e torna-se impraticável para instâncias maiores do problema, dado que o número de subconjuntos cresce exponencialmente com o número de rotas possíveis.

## Solução Aproximada

### Descrição da Solução

Para uma solução mais eficiente, foi adotada uma abordagem aproximada:

1. **Geração de Rotas com BFS Modificado:** O algoritmo de BFS foi ajustado para priorizar a expansão dos nós que atendem os clientes com maior demanda. Esse critério busca maximizar a satisfação das demandas maiores mais rapidamente, potencialmente reduzindo o número total de rotas necessárias.

2. **Geração do Conjunto de Potência e Seleção de Rotas:** Semelhante à abordagem de força bruta, geramos o conjunto de potência das rotas e selecionamos o subconjunto de menor custo que respeita as restrições de demanda.

### Implementação Paralela (Versão 1 - OpenMP)

Para melhorar o desempenho da solução aproximada, foi implementada uma versão paralela da função `find_best_route_parallel`. A diretiva `#pragma omp parallel` é utilizada para criar uma região paralela, onde várias threads podem executar o loop `for` em paralelo. A variável `local_min_cost` é inicializada com o valor máximo inteiro (`INT_MAX`) para cada thread, e a redução é utilizada para encontrar a rota global com o menor custo.

```cpp
std::vector<Path> find_best_route_parallel(const std::vector<std::vector<Path>>& power_set, const Edges& edges, const Demand& demanda) {
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
            if (check_combination_demand(combination, demanda)) {
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
```

#### Benefícios da Paralelização
- Redução de Tempo: A paralelização pode significativamente acelerar a busca pela melhor rota, especialmente quando o conjunto de dados é grande e a carga de trabalho pode ser distribuída efetivamente entre várias threads.

- Eficiência de Recursos: Utilizar múltiplas threads pode ajudar a utilizar mais completamente os recursos da CPU, melhorando a eficiência geral do sistema.
Considerações Técnicas

- Escalonamento e Concorrência: A eficácia da paralelização depende de como as tarefas são escalonadas e da capacidade do hardware. Em alguns casos, pode haver um aumento no overhead devido à gestão de threads e sincronização, o que pode reduzir os ganhos de desempenho.

- Gerenciamento de Memória: Cada thread possui sua própria cópia de variáveis locais, e o uso eficiente da memória pode se tornar um fator crítico, especialmente com grandes volumes de dados.

### Vantagens
Esta abordagem aproximada tende a ser mais rápida que a solução de força bruta, reduzindo o número de rotas e subconjuntos a considerar, devido à priorização inicial das demandas maiores.Porém, não é garantido que a solução gerada será a ótima global, mas sim uma solução próxima ao ótimo.

