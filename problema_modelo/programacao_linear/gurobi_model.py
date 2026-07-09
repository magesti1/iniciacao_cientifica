import os
from gurobipy import Model, GRB, quicksum
import matplotlib.pyplot as plt
import math

def run_optmization(largura_grid, altura_grid, total_rotas, max_voos):
    # Configurações de Grid baseadas na descrição
    largura, altura = largura_grid, altura_grid
    totalPontos = largura * altura
    D = max_voos # Limite de pontos por rota
    # Fator de correção de velocidade do trator
    alpha = 2 # aplha = 2 significa que o trator é 2x mais lento que o drone e alpha = 0.5 significa que é 2x mais rápido
    T_c = 0.5

    num_rotas = total_rotas
    K = range(num_rotas)

    def get_coords(n):
        return n // largura, n % largura

    def isRua(n):
        x, y = get_coords(n)
        return x == 0 or x == altura-1 or y == 0 or y == largura-1 or y == largura // 2

    R = [n for n in range(totalPontos) if isRua(n)]
    V = [n for n in range(totalPontos) if not isRua(n)]
    depot = totalPontos # O ponto teórico 'O'
    nodes = V + R + [depot]

    # Matriz de Distâncias
    dists = {}
    for i in nodes:
        # Ajuste: Coordenadas do depot agora são (-1,-1) para visualização fora do campo
        xi, yi = get_coords(i) if i < totalPontos else (-1, -1)
        for j in nodes:
            # Ajuste: Coordenadas do depot agora são (-1,-1) para visualização fora do campo
            xj, yj = get_coords(j) if j < totalPontos else (-1, -1)

            # REVISÃO: Caso 1: Depot para/de um ponto da rua -> distância 0
            if (i == depot and j in R) or (j == depot and i in R):
                dists[i, j] = 0.0
            # REVISÃO: Caso 2: Ponto da rua para/de outro ponto da rua -> distância de Manhattan
            elif i in R and j in R:
                dists[i, j] = abs(xi - xj) + abs(yi - yj)
            # Caso 3: Depot para/de um ponto de campo (não-Rua) -> distância Euclidiana (comportamento anterior)
            elif i == depot or j == depot:
                # Se chegamos aqui, é depot para/de um ponto de campo (V)
                dists[i, j] = math.sqrt((xi - xj)**2 + (yi - yj)**2)
            # Caso 4: Qualquer outra combinação (Campo-Campo, Campo-Rua, Rua-Campo) -> distância Euclidiana
            else:
                dists[i, j] = math.sqrt((xi - xj)**2 + (yi - yj)**2)


    # CORREÇÃO: Passando o 'env' criado na célula anterior para validar a licença acadêmica
    model = Model("Drone_Routing_Aligned")

    #Limite de tempo para não ultrapassar 30 min
    model.Params.TimeLimit = 3600 # 1800 segundos, 30 min

    arcos = [(i, j, k) for i in nodes for j in nodes for k in K if i != j]
    x = model.addVars(arcos, vtype=GRB.BINARY, name="x")
    f = model.addVars(arcos, vtype=GRB.CONTINUOUS, name="f")

    y = model.addVars([(i, i_prime, k) for i in R for i_prime in R for k in K[:-1]], vtype=GRB.CONTINUOUS, name="y")

    # Função Objetivo: Minimizar sum d_ij * x_ij
    model.setObjective(quicksum(dists[i, j] * x[i, j, k] for i, j, k in arcos) + 
                   quicksum(y[i, i_prime, k] for i in R for i_prime in R for k in K[:-1]), GRB.MINIMIZE)

    for k in K:
        # Restrição: sum sum x_ij <= D (Limite da rota)
        model.addConstr(quicksum(x[i, j, k] for i, j, _k in arcos if _k == k) <= D)

        # Restrição: Sai e chega na rua via depósito O
        model.addConstr(quicksum(x[depot, j, k] for j in R) == 1)
        model.addConstr(quicksum(x[j, depot, k] for j in R) == 1)

        # Conservação de fluxo nos nós visitados
        for j in V + R:
            model.addConstr(quicksum(x[i, j, k] for i in nodes if i != j) ==
                            quicksum(x[j, i, k] for i in nodes if i != j))

    # Restrição: Cobertura exata para pontos de Campo (V)
    for j in V:
        model.addConstr(quicksum(x[i, j, k] for i, _, k in arcos if _ == j) == 1)

    # Variables to store total distance for each drone
    total_drone_dist = model.addVars(K, vtype=GRB.CONTINUOUS, name="total_drone_dist")
    for k in K:
        model.addConstr(total_drone_dist[k] == quicksum(dists[p, q] * x[p, q, k]
                                                        for p in nodes for q in nodes if (p, q, k) in arcos),
                        name=f"calc_total_drone_dist_k_{k}")

    # Define a sufficiently large Big M for implication constraints
    # Max possible distance is from (-1,-1) to (6,6) which is sqrt(7^2+7^2) approx 9.9
    # Max D is 15. So, D * max_dist_per_arc approx 15 * 10 = 150.
    # Let's use 200 as a safe upper bound for M.
    BIG_M_VALUE = 200.0

    # Adicionar a nova restrição: Tempo mínimo que o trator precisa entre pontos i e i' na rua
    # A restrição é: (x[depot, i, k] == 1 AND x[i_prime, depot, k] == 1) => total_drone_dist[k] >= alpha * dists[i, i_prime]
    # para todos i, i_prime em R, e para todos k em K.
    for k in K:
        for i in R:  # 'i' representa o ponto da rua onde o drone k inicia do depot
            for i_prime in R:  # 'i_prime' representa o ponto da rua onde o drone k retorna ao depot
                if i == i_prime:
                    continue # O trator deve viajar entre dois pontos da rua *diferentes*

                # Variável binária auxiliar: premise_var = 1 se o drone k inicia em i E termina em i_prime
                premise_var = model.addVar(vtype=GRB.BINARY, name=f"premise_k{k}_i{i}_iprime{i_prime}")

                # Linearização da condição AND: premise_var == (x[depot, i, k] AND x[i_prime, depot, k])
                # 1. premise_var <= x[depot, i, k]
                model.addConstr(premise_var <= x[depot, i, k],
                                name=f"premise_le1_k{k}_i{i}_iprime{i_prime}")
                # 2. premise_var <= x[i_prime, depot, k]
                model.addConstr(premise_var <= x[i_prime, depot, k],
                                name=f"premise_le2_k{k}_i{i}_iprime{i_prime}")
                # 3. premise_var >= x[depot, i, k] + x[i_prime, depot, k] - 1
                model.addConstr(premise_var >= x[depot, i, k] + x[i_prime, depot, k] - 1,
                                name=f"premise_ge_k{k}_i{i}_iprime{i_prime}")

                # A implicação: SE premise_var == 1 ENTÃO total_drone_dist[k] >= alpha * dists[i, i_prime]
                # Isso é modelado como: total_drone_dist[k] >= alpha * dists[i, i_prime] - BIG_M_VALUE * (1 - premise_var)
                # Só adicionamos esta restrição se houver um custo de distância positivo para o trator cobrir
                if dists[i, i_prime] > 0.0:
                    model.addConstr(total_drone_dist[k] >= alpha * dists[i, i_prime] - BIG_M_VALUE * (1 - premise_var),
                                    name=f"tractor_min_time_k{k}_i{i}_iprime{i_prime}")

    # Restrições de Fluxo (Eliminação de Sub-rotas conforme LaTeX)
    for k in K:
        # Sai com D-1 unidades de carga
        model.addConstr(quicksum(f[depot, j, k] for j in R) == D - 1)

        for i, j, _k in arcos:
            if _k == k:
                model.addConstr(f[i, j, k] <= (D - 1) * x[i, j, k])

        # Balanço: sai 1 unidade em cada ponto visitado
        for i in V + R:
            model.addConstr(quicksum(f[j, i, k] for j in nodes if j != i) -
                            quicksum(f[i, j, k] for j in nodes if i != j) ==
                            quicksum(x[j, i, k] for j in nodes if i != j))
            

        # --- Restries de Tempo do Trator e Carregamento ---

        for k in K[:-1]:
            for i in R: # Ponto onde o drone k TERMINA (chega no depot)
                for i_prime in R: # Ponto onde o drone k+1 INICIA (sai do depot)
                    
                    # Premissa: Drone k termina em i AND Drone k+1 inicia em i_prime
                    trans_var = model.addVar(vtype=GRB.BINARY, name=f"trans_k{k}_i{i}_ip{i_prime}")
                    model.addConstr(trans_var <= x[i, depot, k])
                    model.addConstr(trans_var <= x[depot, i_prime, k+1])
                    model.addConstr(trans_var >= x[i, depot, k] + x[depot, i_prime, k+1] - 1)

                    # Restrio: y deve ser no mnimo o tempo de viagem do trator E o tempo de carregamento
                    # y  max(alpha * dist_ii', T_c)
                    min_gap = max(alpha * dists[i, i_prime], T_c)
                    
                    model.addConstr(y[i, i_prime, k] >= min_gap - BIG_M_VALUE * (1 - trans_var),
                                    name=f"y_gap_min_k{k}_i{i}_ip{i_prime}")


    model.optimize()

    
    notesContent = (
        "Notas do experimento usando:\n"
        f"Largura: {largura_grid}\n"
        f"Altura: {altura_grid}\n"
        f"Número de rotas: {total_rotas}\n"
        f"Máximo de voos: {max_voos}\n"
        "--------------------\n"
    )

    if model.status == GRB.OPTIMAL or model.Status == GRB.TIME_LIMIT:

        if model.Status == GRB.TIME_LIMIT:
            notesContent += f"Não foi possível chegar a solução ótima, mas chegou com um gap de {model.MIPGap * 100:.2f}%\nTempo procurado: {model.Params.TimeLimit}s\n"
        notesContent += f"Custo total otimizado: {model.objVal:.2f}\n"
        notesContent += "\n--- Rotas dos Drones (Arcos Ativos) ---"
        
        for k_idx in K:
            notesContent += f"\nDrone {k_idx}:"
            tem_movimento = False
            for i, j, k_var in x.keys():
                if k_var == k_idx and x[i, j, k_idx].X > 0.5:
                    dist = dists[i, j]
                    if dist > 0:
                        notesContent += f"  De {i} para {j}: Distância {dist:.2f}\n"
                        tem_movimento = True
            if not tem_movimento:
                notesContent += "  Este drone não realizou voos (apenas deslocamento via trator)."

        notesContent += f"\nPontos de Campo (V): {V}\n"
        notesContent += f"Pontos de Estrada (R): {R}\n"
        notesContent += "--- Sequência das Rotas Corrigida ---\n"

        arcos_ativos = {k_idx: {} for k_idx in K}
        for (i, j, k_v), var in x.items():
            if var.X > 0.5:
                if i not in arcos_ativos[k_v]:
                    arcos_ativos[k_v][i] = []
                arcos_ativos[k_v][i].append(j)

        for k_idx in K:
            atual = depot
            rota = [atual]

            for _ in range(D + 5):
                if atual in arcos_ativos[k_idx] and len(arcos_ativos[k_idx][atual]) > 0:
                    proximo = arcos_ativos[k_idx][atual].pop(0)
                    rota.append(proximo)
                    atual = proximo
                    if atual == depot and len(rota) > 1:
                        break
                else:
                    break

            nomes_rota = [str(pt) if pt != depot else "Trator (O)" for pt in rota]
            notesContent += f"Drone {k_idx}: {' -> '.join(nomes_rota)}\n"
    else:
        notesContent += "O modelo não possui uma solução ótima para exibir.\n"

    # Configurações do gráfico
    fig, ax = plt.subplots(figsize=(10, 10))

    for n in range(totalPontos):
        x_coord, y_coord = get_coords(n)
        if n in V:
            ax.scatter(y_coord, x_coord, c='blue', marker='s', s=100, alpha=0.6, label='Campo (V)' if n == V[0] else "")
        else:
            ax.scatter(y_coord, x_coord, c='black', marker='o', s=50, alpha=0.3, label='Estrada (R)' if n == R[0] else "")
        ax.text(y_coord + 0.1, x_coord + 0.1, str(n), fontsize=8, color='black', alpha=0.7)

    if model.status == GRB.OPTIMAL or model.Status == GRB.Status.TIME_LIMIT:
        adj = {k_idx: {} for k_idx in K}
        for (i, j, k_v), var in x.items():
            if var.X > 0.5:
                if i not in adj[k_v]: adj[k_v][i] = []
                adj[k_v][i].append(j)

        colors = ['red', 'green', 'purple', 'orange', 'blue', 'brown']

        for k_idx in K:
            atual = depot
            rota_color = colors[k_idx % len(colors)]

            while atual in adj[k_idx] and len(adj[k_idx][atual]) > 0:
                proximo = adj[k_idx][atual].pop(0)
                xi, yi = get_coords(atual) if atual < totalPontos else (0, 0)
                xj, yj = get_coords(proximo) if proximo < totalPontos else (0, 0)

                is_service = (atual in V or proximo in V)
                line_style = '-' if is_service else '--'
                line_width = 2.5 if is_service else 1.2
                line_alpha = 1.0 if is_service else 0.5

                if not is_service and atual != depot and proximo != depot and xi != xj and yi != yj:
                    ax.plot([yi, yi], [xi, xj], color=rota_color, lw=line_width, ls=line_style, alpha=line_alpha)
                    ax.annotate('', xy=(yj, xj), xytext=(yi, xj),
                                arrowprops=dict(arrowstyle='->', color=rota_color, lw=line_width,
                                                linestyle=line_style, alpha=line_alpha))
                else:
                    ax.annotate('', xy=(yj, xj), xytext=(yi, xi),
                                arrowprops=dict(arrowstyle='->', color=rota_color, lw=line_width,
                                                linestyle=line_style, alpha=line_alpha))
                atual = proximo
                if atual == depot:
                    break

    ax.set_title("Visualização Final: Rotas Completas Extraídas do Modelo")
    ax.set_xlabel("Colunas (Y)")
    ax.set_ylabel("Linhas (X)")
    ax.invert_yaxis()
    ax.grid(True, linestyle=':', alpha=0.4)
    plt.legend(loc='upper right', bbox_to_anchor=(1.2, 1))

    # 3. MUDANÇA AQUI: Criando a pasta usando os parâmetros corretos
    experimentFolderName = f'{largura_grid}_{altura_grid}_{total_rotas}_{max_voos}_solution'
    folderName = os.path.join("solutionsCarregamento", experimentFolderName) 
    imageName = f'{largura_grid}_{altura_grid}_{total_rotas}_{max_voos}.png'
    textName = f'{largura_grid}_{altura_grid}_{total_rotas}_{max_voos}.txt'
    
    os.makedirs(folderName, exist_ok=True)

    imagePath = os.path.join(folderName, imageName)
    plt.savefig(imagePath, dpi=300, bbox_inches='tight')
    plt.close()

    textPath = os.path.join(folderName, textName)
    with open(textPath, 'w', encoding='utf-8') as f:
        f.write(notesContent)
    
    print(f"Pasta das configurações salva como {os.path.realpath(folderName)}")