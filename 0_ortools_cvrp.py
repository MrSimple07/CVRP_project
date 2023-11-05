import networkx as nx
from cvrp_ortools_base import solve_cvrp
from collections import Counter

# Получить матрицу времени перемещений из графа для определенных узлов
def get_time_matrix(G_init):
    G_demand = G_init.copy()
    # Reindex nodes
    i = 0
    index_mapping = {}
    for node in G_demand.nodes:
        index_mapping[node] = i
        i += 1
    G = nx.relabel_nodes(G_demand, index_mapping, copy=False)
    max_index = i

    # Convert to time matrix
    time_matrix = []
    for i in range(max_index):
        l = []
        for j in range(max_index):
            if i == j:
                l.append(0)
            else:
                l.append(round(G[i][j]["weight"], 2))
        time_matrix.append(l)

    return time_matrix, index_mapping

# Получить массив спроса по узлам за период
def calc_demand(G_init, period: list, index_mapping):
    demand_dicts = []
    for date in period:
        demand_dict = nx.get_node_attributes(G_init, date)
        demand_dicts.append(demand_dict)
    # Суммировать спрос за период по каждому узлу
    accum_demand = Counter(demand_dicts[0])
    for i in range(1, len(demand_dicts)):
        accum_demand = accum_demand + Counter(demand_dicts[i])
    # Для использования в or-tools спрос должен быть представлен в виде массива, где узел идентифицируется индексом
    demand_list = list(accum_demand.values())
    # Включить узел депо с нулевым спросом
    start_node_index = index_mapping[start_node]
    demand_list.insert(start_node_index, 0)
    return demand_list

# Граф Василеостровского района со значениями спроса по суткам
G_demand = nx.read_graphml('datasets/graph_with_demand.graphml')
# Даты, за которые рассматривается спрос (в данном варианте - за одну дату)
dates = ['2022-01-05']
# Ограничение количества узлов с потреблением для моделирования проблемы
# ЗДЕСЬ МОГУТ БЫТЬ ПЕРЕЧИСЛЕНЫ ТОЛЬКО УЗЛЫ СО СПРОСОМ
chosen_points = ['113', '47', '15', '87', '200']
# Назначение узла-депо
start_node = '100'
# Задать доступное количество транспортных средств (фиксированное)
num_vehicles = 4
# Из графа исключаются все узлы, кроме выбранных узлов с потреблением и узла-депо
G_demand.remove_nodes_from(list(n for n in G_demand.nodes if n not in chosen_points and n != start_node))

# Получить матрицу времени перемещений для "обрезанного" графа
time_matrix, mapping = get_time_matrix(G_demand)
# Получить спрос за выбранные даты
demand = calc_demand(G_demand, dates, mapping)
# Получить индекс узла депо
depo_index = mapping[start_node]

# Запустить решение CVRP при заданных параметрах
solve_cvrp(time_matrix, demand, depo_index, num_vehicles, mapping)



