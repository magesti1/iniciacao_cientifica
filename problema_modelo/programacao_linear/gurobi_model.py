import os
from gurobipy import Model, GRB, quicksum
import matplotlib.pyplot as plt
import math


# função, que dado número do ponto e largura do grid retorna a componente x e y
def getCoords(n, largura):
        return n // largura, n % largura

def isRua(n, largura, altura):
        x, y = getCoords(n, largura)
        return x == 0 or x == altura-1 or y == 0 or y == largura-1 or y == largura // 2

# funções para cálculo de distância
def isVertical(P, larguraGrid):
    y = P % larguraGrid
    return y == 0 or y == larguraGrid//2 or y == larguraGrid - 1

def isHorizontal(P, larguraGrid, alturaGrid):
    x = P // larguraGrid
    return x == 0 or x == alturaGrid - 1



# Classe principal do problema, fazendo com que todos os dados se organizem de forma melhor, iniciando num único local e inicialização dessas variáveis, incluindo a matriz de distância
class dadosDoProblema:
    def __init__(self, larguraGrid, alturaGrid, totalRotas, maxVoos, firstSolution, alpha, T_c):
        self.largura = larguraGrid
        self.altura = alturaGrid
        self.totalRotas = totalRotas
        self.maxVoos = maxVoos
        self.firstSolution = firstSolution
        self.alpha = alpha
        self.T_c = T_c

        self.K = range(totalRotas)
        self.totalPontos = larguraGrid * alturaGrid

        self.R = [n for n in range(self.totalPontos) if isRua(n, self.largura, self.altura)]
        self.V = [n for n in range(self.totalPontos) if not isRua(n, self.largura, self.altura)]
        self.depot = self.totalPontos
        self.nodes = self.R + self.V + [self.depot]

        self.dists = self.calcularDistancias()

        self.rotas = {}

        self.arcos = [(i, j, k) for i in self.nodes for j in self.nodes for k in self.K if i != j]

    def calcularDistancias(self):
        dists = dict()
        for i in self.nodes:
            xi, yi = getCoords(i, self.largura) if i < self.totalPontos else (-1,-1)
            for j in self.nodes:
                if(i == j):
                    dists[i, j] = 0
                    continue
                xj, yj = getCoords(j, self.largura) if j < self.totalPontos else (-1,-1)
                # Ponto teórico para o campo direto, distância infinita
                if not isRua(i, self.largura, self.altura) and j == self.totalPontos or i == self.totalPontos and not isRua(j, self.largura, self.altura): 
                    dists[i, j] = 10e5
                # Ponto teórico para a rua, distância 0
                elif i == self.totalPontos and isRua(j, self.largura, self.altura) or isRua(i, self.largura, self.altura) and j == self.totalPontos:
                    dists[i, j] = 0
                # Rua para rua, caso especial:
                elif isRua(i, self.largura, self.altura) and isRua(j, self.largura, self.altura):
                    dx, dy = abs(xi-xj), abs(yi-yj)
                    if isHorizontal(i, self.largura, self.altura) and isVertical(j, self.largura) or isVertical(i, self.largura) and isHorizontal(j, self.largura, self.altura):
                        dists[i, j] = dx + dy
                    elif isVertical(i, self.largura) and isVertical(j, self.largura):
                        if dy == 0:
                            dists[i, j] = dx
                            continue
                        c1 = dy + abs(xi - self.altura) + abs(xj - self.altura)
                        c2 = dy + xi + xj
                        dists[i, j] = min(c1, c2)
                    else:
                        if dx == 0:
                            dists[i ,j] = dy
                            continue
                        c1 = dx + abs(yi - self.largura) + abs(yj - self.largura)
                        c2 = dx + yi + yj
                        c3 = dx + abs(yi - (self.largura//2)) + abs(yj - (self.largura//2))
                        dists[i, j] = min(c1, min(c2, c3))
                # Campo para outro ponto, euclidiano
                else:
                    dists[i,j] = math.sqrt((xi - xj)**2 + (yi - yj)**2)
        return dists
        


def runModel(dadosProblema):

    # Criando o modelo
    model = Model("Drone_Routing_Aligned")

    # Definindo as variáveis do modelo
    x = model.addVars(dadosProblema.arcos, vtype=GRB.BINARY, name="x")
    f = model.addVars(dadosProblema.arcos, vtype=GRB.CONTINUOUS, name="f")
    y = model.addVars([(i, i_prime, k) for i in dadosProblema.R for i_prime in dadosProblema.R for k in dadosProblema.K[:-1]], vtype=GRB.CONTINUOUS, name="y")

    # Caso seja uma solução inicial, vai servir como um MIP Start para a próxima solução, então o limite de tempo será menor
    if(dadosProblema.firstSolution):
        model.Params.TimeLimit = 120
    else:
        model.Params.TimeLimit = 300
        
    # Função Objetivo: Minimizar sum d_ij * x_ij
    model.setObjective(quicksum(dadosProblema.dists[i, j] * x[i, j, k] for i, j, k in dadosProblema.arcos) + 
                quicksum(y[i, i_prime, k] for i in dadosProblema.R for i_prime in dadosProblema.R for k in dadosProblema.K[:-1]), GRB.MINIMIZE)

    # Restrições

    for k in dadosProblema.K:
        # 1) Restrição: sum sum x_ij <= D (Limite da rota)
        model.addConstr(quicksum(x[i, j, k] for i, j, _k in dadosProblema.arcos if _k == k) <= dadosProblema.maxVoos)

        # 2, 3) Restrição: Sai e chega na rua via depósito O
        model.addConstr(quicksum(x[dadosProblema.depot, j, k] for j in dadosProblema.R) == 1)
        model.addConstr(quicksum(x[j, dadosProblema.depot, k] for j in dadosProblema.R) == 1)

        # 4) Conservação de fluxo nos nós visitados
        for j in dadosProblema.V + dadosProblema.R:
            model.addConstr(quicksum(x[i, j, k] for i in dadosProblema.nodes if i != j) ==
                            quicksum(x[j, i, k] for i in dadosProblema.nodes if i != j))

    # 5)Restrição: Cobertura exata para pontos de Campo (dadosProblema.V)
    for j in dadosProblema.V:
        model.addConstr(quicksum(x[i, j, k] for i, _, k in dadosProblema.arcos if _ == j) == 1)

    # Variables to store total distance for each drone
    total_drone_dist = model.addVars(dadosProblema.K, vtype=GRB.CONTINUOUS, name="total_drone_dist")
    for k in dadosProblema.K:
        model.addConstr(total_drone_dist[k] == quicksum(dadosProblema.dists[p, q] * x[p, q, k]
                                                        for p in dadosProblema.nodes for q in dadosProblema.nodes if (p, q, k) in dadosProblema.arcos),
                        name=f"calc_total_drone_dist_k_{k}")

    # Define a sufficiently large Big M for implication constraints
    # Max possible distance is from (-1,-1) to (6,6) which is sqrt(7^2+7^2) approx 9.9
    # Max dadosProblema.maxVoos is 15. So, dadosProblema.maxVoos * max_dist_per_arc approx 15 * 10 = 150.
    # Let's use 200 as a safe upper bound for M.
    BIG_M_VALUE = 200.0

    # Adicionar a nova restrição: Tempo mínimo que o trator precisa entre pontos i e i' na rua
    # A restrição é: (x[dadosProblema.depot, i, k] == 1 AND x[i_prime, dadosProblema.depot, k] == 1) => total_drone_dist[k] >= alpha * dadosProblema.dists[i, i_prime]
    # para todos i, i_prime em dadosProblema.R, e para todos k em dadosProblema.K.
    for k in dadosProblema.K:
        for i in dadosProblema.R:  # 'i' representa o ponto da rua onde o drone k inicia do dadosProblema.depot
            for i_prime in dadosProblema.R:  # 'i_prime' representa o ponto da rua onde o drone k retorna ao dadosProblema.depot
                if i == i_prime:
                    continue # O trator deve viajar entre dois pontos da rua *diferentes*

                # Variável binária auxiliar: premise_var = 1 se o drone k inicia em i E termina em i_prime
                premise_var = model.addVar(vtype=GRB.BINARY, name=f"premise_k{k}_i{i}_iprime{i_prime}")

                # Linearização da condição AND: premise_var == (x[dadosProblema.depot, i, k] AND x[i_prime, dadosProblema.depot, k])
                # 1. premise_var <= x[dadosProblema.depot, i, k]
                model.addConstr(premise_var <= x[dadosProblema.depot, i, k],
                                name=f"premise_le1_k{k}_i{i}_iprime{i_prime}")
                # 2. premise_var <= x[i_prime, dadosProblema.depot, k]
                model.addConstr(premise_var <= x[i_prime, dadosProblema.depot, k],
                                name=f"premise_le2_k{k}_i{i}_iprime{i_prime}")
                # 3. premise_var >= x[dadosProblema.depot, i, k] + x[i_prime, dadosProblema.depot, k] - 1
                model.addConstr(premise_var >= x[dadosProblema.depot, i, k] + x[i_prime, dadosProblema.depot, k] - 1,
                                name=f"premise_ge_k{k}_i{i}_iprime{i_prime}")

                # A implicação: SE premise_var == 1 ENTÃO total_drone_dist[k] >= alpha * dadosProblema.dists[i, i_prime]
                # Isso é modelado como: total_drone_dist[k] >= alpha * dadosProblema.dists[i, i_prime] - BIG_M_VALUE * (1 - premise_var)
                # Só adicionamos esta restrição se houver um custo de distância positivo para o trator cobrir
                if dadosProblema.dists[i, i_prime] > 0.0:
                    model.addConstr(total_drone_dist[k] >= dadosProblema.alpha * dadosProblema.dists[i, i_prime] - BIG_M_VALUE * (1 - premise_var),
                                    name=f"tractor_min_time_k{k}_i{i}_iprime{i_prime}")

    # Restrições de Fluxo (Eliminação de Sub-rotas conforme LaTeX)
    for k in dadosProblema.K:
        # Sai com dadosProblema.maxVoos-1 unidades de carga
        model.addConstr(quicksum(f[dadosProblema.depot, j, k] for j in dadosProblema.R) == dadosProblema.maxVoos - 1)

        for i, j, _k in dadosProblema.arcos:
            if _k == k:
                model.addConstr(f[i, j, k] <= (dadosProblema.maxVoos - 1) * x[i, j, k])

        # Balanço: sai 1 unidade em cada ponto visitado
        for i in dadosProblema.V + dadosProblema.R:
            model.addConstr(quicksum(f[j, i, k] for j in dadosProblema.nodes if j != i) -
                            quicksum(f[i, j, k] for j in dadosProblema.nodes if i != j) ==
                            quicksum(x[j, i, k] for j in dadosProblema.nodes if i != j))
            

        # --- Restries de Tempo do Trator e Carregamento ---

    for k in dadosProblema.K[:-1]:
        for i in dadosProblema.R: # Ponto onde o drone k TERMINA (chega no dadosProblema.depot)
            for i_prime in dadosProblema.R: # Ponto onde o drone k+1 INICIA (sai do dadosProblema.depot)
                
                # Premissa: Drone k termina em i AND Drone k+1 inicia em i_prime
                trans_var = model.addVar(vtype=GRB.BINARY, name=f"trans_k{k}_i{i}_ip{i_prime}")
                model.addConstr(trans_var <= x[i, dadosProblema.depot, k])
                model.addConstr(trans_var <= x[dadosProblema.depot, i_prime, k+1])
                model.addConstr(trans_var >= x[i, dadosProblema.depot, k] + x[dadosProblema.depot, i_prime, k+1] - 1)

                # Restrio: y deve ser no mnimo o tempo de viagem do trator E o tempo de carregamento
                # y  max(dadosProblema.alpha * dist_ii', T_c)
                min_gap = max(dadosProblema.alpha * dadosProblema.dists[i, i_prime], dadosProblema.T_c)
                
                model.addConstr(y[i, i_prime, k] >= min_gap - BIG_M_VALUE * (1 - trans_var),
                                name=f"y_gap_min_k{k}_i{i}_ip{i_prime}")

    model.update() 
    if (not dadosProblema.firstSolution) and os.path.exists("solucao_inicial.sol"): 
        model.read("solucao_inicial.sol")

    model.optimize()

    if dadosProblema.firstSolution and model.SolCount > 0:
        model.write("solucao_inicial.sol")

    return model, x


def geraArquivoTexto(dadosProblema, model, folderName, x):
    notesContent = (
            "Notas do experimento usando:\n"
            f"Largura: {dadosProblema.largura}\n"
            f"Altura: {dadosProblema.altura}\n"
            f"Número de rotas: {dadosProblema.totalRotas}\n"
            f"Máximo de voos: {dadosProblema.maxVoos}\n"
            f"Tempo de carregamento: {dadosProblema.T_c}\n"
            f"Alpha: {dadosProblema.alpha}\n"
            f"Foi utilizada a busca 2-opt no MIP Start apenas"
            "--------------------\n\n"
        )

    if model.status != GRB.OPTIMAL and model.status != GRB.TIME_LIMIT:
        notesContent += ( "Não foi possível encontrar nenhuma solução viável no tempo estipulado.\n")
        return False

    if model.status == GRB.TIME_LIMIT:
        notesContent += f"Não foi possível chegar a solução ótima, conseguindo um gap de {model.MIPGAP *100:.2f}%\n"
        notesContent += f"Tempo procurado: {model.Params.TimeLimit}\n"

    notesContent += f"Custo total otimizado: {model.objVal:.2f}\n"

    notesContent += f"\n----Sequência de rota dos drones----\n"

    arcos_ativos = {k_idx: {} for k_idx in dadosProblema.K}
    for (i, j, k_v), var in x.items():
        if var.X > 0.5:
            if i not in arcos_ativos[k_v]:
                arcos_ativos[k_v][i] = []
            arcos_ativos[k_v][i].append(j)

    for k_idx in dadosProblema.K:
        atual = dadosProblema.depot
        rota = [atual]

        for _ in range(dadosProblema.maxVoos + 5):
            if atual in arcos_ativos[k_idx] and len(arcos_ativos[k_idx][atual]) > 0:
                proximo = arcos_ativos[k_idx][atual].pop(0)
                rota.append(proximo)
                atual = proximo
                if atual == dadosProblema.depot and len(rota) > 1:
                    break
            else:
                break

        nomes_rota = [str(pt) if pt != dadosProblema.depot else "Trator (O)" for pt in rota]
        notesContent += f"Drone {k_idx}: {' -> '.join(nomes_rota)}\n"

    textName = f'{dadosProblema.largura}_{dadosProblema.altura}_{dadosProblema.totalRotas}_{dadosProblema.maxVoos}.txt'

    textPath = os.path.join(folderName, textName)
    with open(textPath, 'w', encoding='utf-8') as f:
        f.write(notesContent)

    print(f"Pasta das configurações salva como {os.path.realpath(folderName)}")


def geraImagemRotas(dadosProblema, model, folderName, x):
    fig, ax = plt.subplots(figsize=(10, 10))
    
    for n in range(dadosProblema.totalPontos):
        x_coord, y_coord = getCoords(n, dadosProblema.largura)
        if n in dadosProblema.V:
            ax.scatter(y_coord, x_coord, c='blue', marker='s', s=100, alpha=0.6, label='Campo (V)' if n == dadosProblema.V[0] else "")
        else:
            ax.scatter(y_coord, x_coord, c='black', marker='o', s=50, alpha=0.3, label='Estrada (R)' if n == dadosProblema.R[0] else "")
        ax.text(y_coord + 0.1, x_coord + 0.1, str(n), fontsize=8, color='black', alpha=0.7)

    if model.status == GRB.OPTIMAL or model.Status == GRB.Status.TIME_LIMIT:
        adj = {k_idx: {} for k_idx in dadosProblema.K}
        for (i, j, k_v), var in x.items():
            if var.X > 0.5:
                if i not in adj[k_v]: adj[k_v][i] = []
                adj[k_v][i].append(j)

        colors = ['red', 'blue', 'green', 'yellow', 'orange', 'purple', 'pink', 'brown', 'cyan', 'magenta', 'lime', 'teal', 'navy', 'gold', 'coral', 'indigo', 'violet', 'turquoise', 'crimson', 'olive']

        for k_idx in dadosProblema.K:
            atual = dadosProblema.depot
            rota_color = colors[k_idx % len(colors)]

            while atual in adj[k_idx] and len(adj[k_idx][atual]) > 0:
                proximo = adj[k_idx][atual].pop(0)
                xi, yi = getCoords(atual, dadosProblema.largura) if atual < dadosProblema.totalPontos else (0, 0)
                xj, yj = getCoords(proximo, dadosProblema.largura) if proximo < dadosProblema.totalPontos else (0, 0)

                is_service = (atual in dadosProblema.V or proximo in dadosProblema.V)
                line_style = '-' if is_service else '--'
                line_width = 2.5 if is_service else 1.2
                line_alpha = 1.0 if is_service else 0.5

                if not is_service and atual != dadosProblema.depot and proximo != dadosProblema.depot and xi != xj and yi != yj:
                    ax.plot([yi, yi], [xi, xj], color=rota_color, lw=line_width, ls=line_style, alpha=line_alpha)
                    ax.annotate('', xy=(yj, xj), xytext=(yi, xj),
                                arrowprops=dict(arrowstyle='->', color=rota_color, lw=line_width,
                                                linestyle=line_style, alpha=line_alpha))
                else:
                    ax.annotate('', xy=(yj, xj), xytext=(yi, xi),
                                arrowprops=dict(arrowstyle='->', color=rota_color, lw=line_width,
                                                linestyle=line_style, alpha=line_alpha))
                atual = proximo
                if atual == dadosProblema.depot:
                    break

    ax.set_title("Visualização Final: Rotas Completas Extraídas do Modelo")
    ax.set_xlabel("Colunas (Y)")
    ax.set_ylabel("Linhas (X)")
    ax.invert_yaxis()
    ax.grid(True, linestyle=':', alpha=0.4)
    plt.legend(loc='upper right', bbox_to_anchor=(1.2, 1))

    imageName = f'{dadosProblema.largura}_{dadosProblema.altura}_{dadosProblema.totalRotas}_{dadosProblema.maxVoos}.png'
    imagePath = os.path.join(folderName, imageName)
    plt.savefig(imagePath, dpi=300, bbox_inches='tight')
    plt.close()
