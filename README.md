# Vehicle Routing Problem (VRP) Solver README

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

### Vantagens

Esta abordagem aproximada tende a ser mais rápida que a solução de força bruta, reduzindo o número de rotas e subconjuntos a considerar, devido à priorização inicial das demandas maiores.Porém, não é garantido que a solução gerada será a ótima global, mas sim uma solução próxima ao ótimo.

