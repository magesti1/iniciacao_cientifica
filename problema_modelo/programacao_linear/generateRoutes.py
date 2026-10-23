import gurobipy as gp
import copy

def calculateDist(p1, p2, largura):
    x1, y1 = p1 % largura, p1 // largura
    x2, y2 = p2 % largura, p2 // largura
    return abs(x1 - x2) + abs(y1 - y2)

def recalcuteDists(dadosProblema):
    matriz = copy.deepcopy(dadosProblema.dists)
    for p in dadosProblema.V:
        divInteira = p // dadosProblema.largura
        mod = p % dadosProblema.largura
        p1 = mod
        p2 = (dadosProblema.altura-1) * dadosProblema.largura + mod
        p3 = divInteira * dadosProblema.largura
        p4 = ((divInteira + 1) * dadosProblema.largura) - 1
        p5 = (divInteira * dadosProblema.largura) + dadosProblema.largura // 2
        d1 = calculateDist(p, p1, dadosProblema.largura)
        d2 = calculateDist(p, p2, dadosProblema.largura)
        d3 = calculateDist(p, p3, dadosProblema.largura)
        d4 = calculateDist(p, p4, dadosProblema.largura)
        d5 = calculateDist(p, p5, dadosProblema.largura)
        minDist = min(d1, d2, d3, d4, d5)
        matriz[p, dadosProblema.totalPontos] = minDist
        matriz[dadosProblema.totalPontos, p] = minDist

    return matriz


def geraRotas(dadosProblema):

    n = len(dadosProblema.V) 
    depot = dadosProblema.totalPontos

    nos = dadosProblema.V + [depot]

    matrizCustos = recalcuteDists(dadosProblema)
    
    modeloGerador = gp.Model("Gerador_Rotas")

    # Variáveis de decisão
    typeX = {}
    for i in nos:
        for j in nos:
            if i != j:
                typeX[i, j] = 0

    # Variável x
    x = modeloGerador.addVars(typeX, vtype=gp.GRB.BINARY, name='x')
    # Variável f para restrição de fluxo
    f = modeloGerador.addVars(typeX, vtype=gp.GRB.INTEGER, lb=0, name='f')

    # Função objetivo (Não queremos minimizar nada)
    modeloGerador.setObjective(0)

    # Parâmetros
    # Esses parâmetros vão buscar todas soluções possíveis
    modeloGerador.setParam(gp.GRB.Param.PoolSolutions, 2147483647)
    modeloGerador.setParam(gp.GRB.Param.PoolSearchMode, 2)

    modeloGerador.Params.OutputFlag = 0

    # Limite de pontos
    
    modeloGerador.addConstr(
        gp.quicksum(x[depot, j] for j in dadosProblema.V) == 1,
        name='sair_depot'
    )

    modeloGerador.addConstr(
        gp.quicksum(x[i, depot] for i in dadosProblema.V) == 1,
        name='chegar_depot'
    )

    # Sair e chegar de todo nó
    modeloGerador.addConstrs(
        (gp.quicksum(x[j, i] for j in nos if i != j) ==
        gp.quicksum(x[i, j] for j in nos if i != j)
        for i in nos),
        name='sair_chegar_todos_nos' 
    )

    # Cada ponto pode ser visitado no máximo 1 vez
    modeloGerador.addConstrs(
        gp.quicksum(x[j, i] for j in nos if i != j) <= 1 for i in dadosProblema.V
    )

    modeloGerador.addConstr(
        gp.quicksum(x[j, i] for i in dadosProblema.V for j in nos if i != j) <= dadosProblema.maxVoos,
        name='limite_max_pontos'
    )

    # Restrições de fluxo
    modeloGerador.addConstr(
        gp.quicksum(f[depot, j] for j in nos if j != depot) == gp.quicksum(x[j, i] for i in dadosProblema.V for j in nos if i != j),
        name='sair_com_carga_n-1'
    ) 

    # Conserva o fluxo conforme visita os nós
    modeloGerador.addConstrs(
        (gp.quicksum(f[j, i] for j in nos if i != j) -
        gp.quicksum(f[i, j] for j in nos if i != j) == gp.quicksum(x[j, i] for j in nos if i != j)
        for i in dadosProblema.V),
        name='conserva_fluxo'
    )

    # Só conserva nos arcos visitados
    modeloGerador.addConstrs(
        (f[i,j] <= (n) * x[i, j]
        for i,j in typeX),
        name='fluxo_apenas_onde_passar'
    )
    
    
    modeloGerador.optimize()

    print(modeloGerador.SolCount)
    operations = []

    
    for k in range(modeloGerador.SolCount):
        rota = []
        custo = 0
        noAtual = depot
        modeloGerador.Params.SolutionNumber = k
        while(True):
            for j in nos:
                if noAtual != j and x[noAtual, j].Xn > 0.5:
                    custo += matrizCustos[noAtual, j]
                    noAtual = j
                    if(j != depot): rota.append(noAtual)
            if(noAtual == depot): break

        operations.append(
            {
                "nodes": rota,
                "cost": custo
            }
        )

    print(operations)