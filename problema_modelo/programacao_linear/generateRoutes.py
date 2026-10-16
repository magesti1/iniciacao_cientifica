import gurobipy as gp

def geraRotas(dadosProblema):

    n = len(dadosProblema.V)
    depot = dadosProblema.V[0]

    modeloGerador = gp.Model("Gerador_Rotas")

    # Variáveis de decisão
    typeX = {}
    for i in dadosProblema.V:
        for j in dadosProblema.V:
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
    modeloGerador.setParam(gp.GRB.Param.PoolSolutions, 1000)
    modeloGerador.setParam(gp.GRB.Param.PoolSearchMode, 2)

    # Limite de pontos
    '''
    modeloGerador.addConstr(
        gp.quicksum(x[i, j] for i, j in typeX) <= dadosProblema.maxVoos
        )
    '''

    # Sair e chegar de todo nó
    modeloGerador.addConstrs(
        gp.quicksum(x[i, j] for i in dadosProblema.V if i != j) == 1
        for j in dadosProblema.V 
    )

    
    modeloGerador.addConstrs(
        gp.quicksum(x[i, j] for j in dadosProblema.V if i != j) == 1
        for i in dadosProblema.V 
    )

    # Restrições de fluxo
    # Parte com carga |V|-1
    modeloGerador.addConstr(
        gp.quicksum(f[depot, j] for j in dadosProblema.V if j != depot) == n - 1
        ) 

    # Conserva o fluxo conforme visita os nós
    modeloGerador.addConstrs(
        gp.quicksum(f[j, i] for j in dadosProblema.V if i != j) -
        gp.quicksum(f[i, j] for j in dadosProblema.V if i != j) == 1
        for i in dadosProblema.V if i != depot
    )

    # Só conserva nos arcos visitados
    modeloGerador.addConstrs(
        f[i,j] <= (n-1) * x[i, j]
        for i,j in typeX
    )

    
    modeloGerador.optimize()

    print(modeloGerador.SolCount)

    for i, j in typeX:
        if(x[i, j].X > 0.5):
            print(i, j, x[i, j].X)