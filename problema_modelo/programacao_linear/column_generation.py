import gurobipy as gp
from generateRoutes import recalcuteDists

droneRoutes = []
R_d = list(range(len(droneRoutes)))
c = [
        droneRoutes[r]["cost"]
        for r in R_d
    ]

delta = {}

def criarMestre(dadosProblema):
    V = dadosProblema.V
    nos = V + [dadosProblema.depot]
    for r in R_d:
        for i in nos:
            delta[r, i] = 1



    mestre = gp.Model("master_problem")
    # Adicionando variáveis
    # Escolha das rotas dos drones
    y = mestre.addVars(
        R_d,
        vtype=gp.GRB.CONTINUOUS,
        name="y",
        lb=0,
    )

    # Variável artificial
    mi = mestre.addVars(V, vtype=gp.GRB.CONTINUOUS, name='mi')

    mestre.setObjective(

        gp.quicksum(
            c[r] * y[r]
            for r in R_d
        ) + 50* gp.quicksum(mi[i] for i in V),

        sense=gp.GRB.MINIMIZE
    )

    mestre.Params.OutputFlag = 0

    restricaoPi = mestre.addConstrs(

        (
            gp.quicksum(delta[r, i] * y[r] for r in R_d) + mi[i]>= 1
            for i in V
        ),

        name="cover"
    )

    return mestre, restricaoPi


def resolverSubproblema(pi, dadosProblema, matrizCustos):
    # Dados do subproblema
    V = dadosProblema.V
    depot = dadosProblema.totalPontos
    nos = V + [depot]
    
    # Modelo
    subProblem = gp.Model("subproblema")

    # Variáveis
    typeX = {}
    for i in nos:
        for j in nos:
            if i != j:
                typeX[i, j] = 0

    x = subProblem.addVars(
        typeX, 
        vtype=gp.GRB.BINARY,
        name='x'
    )

    # Variável f para controlar o fluxo
    f = subProblem.addVars(typeX, vtype=gp.GRB.INTEGER, lb=0, name='f')

    # Parâmetos
    #subProblem.setParam(gp.GRB.Param.PoolSolutions, 1000)
    #subProblem.setParam(gp.GRB.Param.PoolSearchMode, 2)
    subProblem.Params.OutputFlag = 0

    # Função objetivo
    subProblem.setObjective(
    gp.quicksum(matrizCustos[i, j] * x[i, j]for i in nos for j in nos if i != j) -
    gp.quicksum(pi[i] * gp.quicksum(x[j, i] for j in nos if j != i) for i in V),
    sense=gp.GRB.MINIMIZE
    )
    # Restrições 
    # Sair depot
    subProblem.addConstr(gp.quicksum(x[depot, j] for j in V) == 1,
                         name='sair_depot')

    # Chegar depot
    subProblem.addConstr(gp.quicksum(x[j,depot] for j in V) == 1,
                         name='chegar_depot')

    # Sair e chegar de todo nó
    subProblem.addConstrs(
            (gp.quicksum(x[j, i] for j in nos if i != j) ==
            gp.quicksum(x[i, j] for j in nos if i != j)
            for i in nos),
            name='sair_chegar_todos_nos' 
    )

    # Máximo 1 aresta incidindo em cada nó do campo
    subProblem.addConstrs(
        gp.quicksum(x[j, i] for j in nos if i != j) <= 1 for i in V
    )

    # Limite de pontos
    subProblem.addConstr(
            gp.quicksum(x[j, i] for i in V for j in nos if i != j) <= dadosProblema.maxVoos,
            name='limite_max_pontos'
    )

    # Restrições de fluxo
    subProblem.addConstr(
            gp.quicksum(f[depot, j] for j in nos if j != depot) == (dadosProblema.maxVoos-1),
            name='sair_com_carga_n-1'
    ) 

    subProblem.addConstrs(
            (gp.quicksum(f[j, i] for j in nos if i != j) -
            gp.quicksum(f[i, j] for j in nos if i != j) == gp.quicksum(x[j, i] for j in nos if i != j)
            for i in V),
            name='conserva_fluxo'
    )

    subProblem.addConstrs(
            (f[i,j] <= (dadosProblema.maxVoos-1) * x[i, j]
            for i,j in typeX),
            name='fluxo_apenas_onde_passar'
    )

    subProblem.optimize()

    #print(subProblem.SolCount)

    #for k in range(subProblem.SolCount):
    #        subProblem.Params.SolutionNumber = k
    #        print(subProblem.PoolObjVal)
    #        for i,j in typeX:
    #            if(x[i,j].Xn > 0.5):
    #                print(i, j, int(f[i,j].Xn))
    #        print("-"*20)

    if subProblem.SolCount > 0 and subProblem.PoolObjVal < -1e-6:
        #subProblem.Params.SolutionNumber = subProblem.SolCount-1
        rota = [depot]
        noAtual = depot
        custo = 0
        while(True):
                    for j in nos:
                        if noAtual != j and x[noAtual, j].X > 0.5:
                            custo += matrizCustos[noAtual, j]
                            noAtual = j
                            if(j != depot): rota.append(noAtual)
                    if(noAtual == depot): break
        #print(rota)
        
        droneRoutes.append({'nodes': rota, 'cost': custo})
        c.append(droneRoutes[len(R_d)]['cost'])

        for i in nos:
            delta[len(R_d), i] = int(i in rota)

    #print(delta)

    #print(droneRoutes, c, R_d, subProblem.PoolObjVal)
        
    return subProblem.Objval, len(R_d)


def columnGeneration(dadosProblema):
    # Dados específicos
    V = dadosProblema.V
    custos = recalcuteDists(dadosProblema)
    nos = V + [dadosProblema.depot]

    # Criando o problema mestre
    problemaMestre, pi = criarMestre(dadosProblema)

    iteracao = 0

    while True:
        iteracao += 1

        problemaMestre.optimize()

        duaisPi = {i: pi[i].Pi for i in V}
        custoReduzido, novaRota = resolverSubproblema(duaisPi, dadosProblema, custos)

        if(custoReduzido >= -1e-6):
            print(f"Solução Ótima encontrada!")
            break
        R_d.append(novaRota)
        
        coefs = [delta[novaRota, i] for i in V]
        constrs = [pi[piConstr] for piConstr in pi]

        custoColuna = droneRoutes[novaRota]['cost']
        
        coluna = gp.Column(coefs, constrs)

        problemaMestre.addVar(
            lb = 0,
            obj = custoColuna,
            vtype = gp.GRB.CONTINUOUS,
            column = coluna,
            name=f'y[{novaRota}]'
        )

        teste = problemaMestre.getConstrs()

        vars = problemaMestre.getVars()
        print(custoReduzido)
        print(vars)
        print(coefs)
        print(constrs)
        for i in range(len(V)):
            print(teste[i].Pi)
        print(duaisPi)
        #print(vars)
        #print(duaisPi)
        print(f"Iteração {iteracao:02d} | Custo Relaxado: {problemaMestre.ObjVal:.2f} | Nova Coluna: {droneRoutes[novaRota]}\n")

    print(f"Custo Relaxado: {problemaMestre.ObjVal:.2f}")
    print(vars)
