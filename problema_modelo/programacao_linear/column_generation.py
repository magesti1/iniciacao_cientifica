import gurobipy as gp
from generateRoutes import recalcuteDists

droneRoutes = []
R_d = list(range(len(droneRoutes)))
c = [
        droneRoutes[r]["cost"]
        for r in R_d
    ]
def criarMestre(dadosProblema):
    V = dadosProblema.V
    nos = V + [dadosProblema.depot]

    delta = {}
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
        lb=0.5,
    )

    # Variável artificial
    mi = mestre.addVar(vtype=gp.GRB.CONTINUOUS, name='mi')

    mestre.setObjective(

        gp.quicksum(
            c[r] * y[r]
            for r in R_d
        ) + 10e3*mi,

        sense=gp.GRB.MINIMIZE
    )

    mestre.Params.OutputFlag = 0

    restricaoPi = mestre.addConstrs(

        (
            gp.quicksum(delta[r, i] * y[r] for r in R_d) + mi>= 1
            for i in nos
        ),

        name="cover"
    )

    return mestre, restricaoPi, y


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
    subProblem.setParam(gp.GRB.Param.PoolSolutions, 1000)
    subProblem.setParam(gp.GRB.Param.PoolSearchMode, 2)
    subProblem.Params.OutputFlag = 0

    # Função objetivo
    subProblem.setObjective(gp.quicksum(matrizCustos[i,j] * x[i,j] for i in nos for j in nos if i != j)
                            - gp.quicksum(matrizCustos[i,j] * pi[j] for i in nos for j in nos),
                            sense=gp.GRB.MINIMIZE)
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

    print(subProblem.SolCount)

    for k in range(subProblem.SolCount):
            subProblem.Params.SolutionNumber = k
            print(subProblem.PoolObjVal)
            for i,j in typeX:
                if(x[i,j].Xn > 0.5):
                    print(i, j, int(f[i,j].Xn))
            print("-"*20)

    return subProblem.ObjVal


def columnGeneration(dadosProblema):
    # Dados específicos
    V = dadosProblema.V
    custos = recalcuteDists(dadosProblema)
    nos = V + [dadosProblema.depot]

    # Criando o problema mestre
    problemaMestre, pi, y = criarMestre(dadosProblema)

    iteracao = 0
    while True:
        iteracao += 1
        problemaMestre.optimize()
        duaisPi = {i: pi[i].Pi for i in nos}
        print(y)
        print(problemaMestre.ObjVal)
        print(duaisPi)
        c_r = resolverSubproblema(duaisPi, dadosProblema, custos)
        #if(c_r >= -1e-6):
        #    print("Solução Ótima encontrada!")
        #   break

        break