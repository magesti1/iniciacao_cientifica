import os
from gurobipy import Model, GRB, quicksum
import matplotlib.pyplot as plt
import math

def run_optmization(largura_grid, altura_grid, total_rotas, max_voos):
    # Configurações de Grid baseadas na descrição
    largura, altura = largura_grid, altura_grid
    totalPontos = largura * altura
    D = max_voos # Limite de pontos por rota
    num_rotas = total_rotas
    K = range(num_rotas)

    def get_coords(n):
        return n // largura, n % largura

    def isRua(n):
        cx, cy = get_coords(n)
        return cx == 0 or cx == altura-1 or cy == 0 or cy == largura-1 or cy == largura // 2

    R = [n for n in range(totalPontos) if isRua(n)]
    V = [n for n in range(totalPontos) if not isRua(n)]
    depot = totalPontos # O ponto teórico 'O'
    nodes = V + R + [depot]

    # Matriz de Distâncias
    dists = {}
    for i in nodes:
        xi, yi = get_coords(i) if i < totalPontos else (0, 0)
        for j in nodes:
            xj, yj = get_coords(j) if j < totalPontos else (0, 0)
            if i in R and j in R: 
                dists[i, j] = 0.0
            elif i == depot or j == depot:
                dists[i, j] = (abs(xi-xj) + abs(yi-yj)) if (i in R or j in R) else math.sqrt((xi-xj)**2 + (yi-yj)**2)
            else: 
                dists[i, j] = math.sqrt((xi-xj)**2 + (yi-yj)**2)

    model = Model("Drone_Routing_Aligned")

    arcos = [(i, j, k) for i in nodes for j in nodes for k in K if i != j]
    
    # 'x' e 'k' matemáticos mantidos em segurança aqui
    x = model.addVars(arcos, vtype=GRB.BINARY, name="x")
    f = model.addVars(arcos, vtype=GRB.CONTINUOUS, name="f")

    # Função Objetivo
    model.setObjective(quicksum(dists[i, j] * x[i, j, k] for i, j, k in arcos), GRB.MINIMIZE)

    for k_idx in K:
        model.addConstr(quicksum(x[i, j, k_idx] for i, j, _k in arcos if _k == k_idx) <= D)
        model.addConstr(quicksum(x[depot, j, k_idx] for j in R) == 1)
        model.addConstr(quicksum(x[j, depot, k_idx] for j in R) == 1)

        for j in V + R:
            model.addConstr(quicksum(x[i, j, k_idx] for i in nodes if i != j) ==
                            quicksum(x[j, i, k_idx] for i in nodes if i != j))

    for j in V:
        model.addConstr(quicksum(x[i, j, k_idx] for i, _, k_idx in arcos if _ == j) == 1)

    for k_idx in K:
        model.addConstr(quicksum(f[depot, j, k_idx] for j in R) == D - 1)
        for i, j, _k in arcos:
            if _k == k_idx:
                model.addConstr(f[i, j, k_idx] <= (D - 1) * x[i, j, k_idx])

        for i in V + R:
            model.addConstr(quicksum(f[j, i, k_idx] for j in nodes if j != i) -
                            quicksum(f[i, j, k_idx] for j in nodes if i != j) ==
                            quicksum(x[j, i, k_idx] for j in nodes if i != j))

    model.optimize()

    
    notesContent = (
        "Notas do experimento usando:\n"
        f"Largura: {largura_grid}\n"
        f"Altura: {altura_grid}\n"
        f"Número de rotas: {total_rotas}\n"
        f"Máximo de voos: {max_voos}\n"
        "--------------------\n"
    )

    if model.status == GRB.OPTIMAL:
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

    if model.status == GRB.OPTIMAL:
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
    folderName = os.path.join("solutions", experimentFolderName) 
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