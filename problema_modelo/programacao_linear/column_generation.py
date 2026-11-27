import gurobipy as gp
from generateRoutes import recalcuteDists
from itertools import permutations

droneRoutes = []
R_d = list(range(len(droneRoutes)))
c = [
        droneRoutes[r]["cost"]
        for r in R_d
    ]

delta = {}
start = {}
finish = {}

def criarMestre(dadosProblema):
    V = dadosProblema.V
    nos = V + [dadosProblema.depot]
    for r in R_d:
        for i in nos:
            delta[r, i] = 1

    mestre = gp.Model("master_problem")

    s_pos = mestre.addVars(dadosProblema.R, vtype=gp.GRB.CONTINUOUS, lb=0, name='s_pos')
    s_neg = mestre.addVars(dadosProblema.R, vtype=gp.GRB.CONTINUOUS, lb=0, name='s_neg')


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

    mestre.Params.TimeLimit = 1800

    mestre.setObjective(

        gp.quicksum(
            c[r] * y[r]
            for r in R_d
        ) + 500000* gp.quicksum(mi[i] for i in V)
          + 10000 * gp.quicksum(s_pos[i] + s_neg[i] for i in dadosProblema.R),

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

    restricaoSigma = mestre.addConstrs(
        (
            gp.quicksum(y[r] * (start[r, i] - finish[r, i]) for r in R_d) + s_pos[i] - s_neg[i] == 0
            for i in dadosProblema.R
        )
    )
    

    return mestre, restricaoPi, restricaoSigma


def resolverSubproblema(pi, sigma, dadosProblema, matrizCustos):
    # Dados do subproblema
    R = dadosProblema.R
    V = dadosProblema.V
    depot = dadosProblema.totalPontos
    nos = R + V + [depot]
    alpha = dadosProblema.alpha
    
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

    # Parâmetos
    subProblem.Params.OutputFlag = 0
    subProblem.Params.LazyConstraints = 1
    subProblem.Params.TimeLimit = 120
    subProblem.Params.PoolSolutions = 100

    # Função objetivo
    subProblem.setObjective(
        gp.quicksum(matrizCustos[i, j] * x[i, j]for i in nos for j in nos if i != j) -
        gp.quicksum(pi[i] * gp.quicksum(x[j, i] for j in nos if j != i) for i in V) -
        gp.quicksum(sigma[i] * (x[depot, i] - x[i, depot]) for i in R),
        sense=gp.GRB.MINIMIZE
    )
    # Restrições 
    # Sair depot
    subProblem.addConstr(gp.quicksum(x[depot, j] for j in R) == 1,
                         name='sair_depot')

    # Chegar depot
    subProblem.addConstr(gp.quicksum(x[j,depot] for j in R) == 1,
                         name='chegar_depot')

    # Sair e chegar de todo nó
    subProblem.addConstrs(
            (gp.quicksum(x[j, i] for j in nos if i != j) ==
            gp.quicksum(x[i, j] for j in nos if i != j)
            for i in nos),
            name='sair_chegar_todos_nos' 
    )

    # Máximo 1 aresta incidindo em cada nó 
    subProblem.addConstrs(
        gp.quicksum(x[j, i] for j in nos if i != j) <= 1 for i in nos
    )


    # Limite de pontos
    subProblem.addConstr(
            gp.quicksum(matrizCustos[i,j] * x[j, i] for i in nos for j in nos if i != j) <= dadosProblema.maxDistancia,
            name='limite_max_pontos'
    )

    distTotal = subProblem.addVar(lb=0, name='distTotal')
    subProblem.addConstr(
        distTotal == gp.quicksum(matrizCustos[i, j] * x[i, j] for i, j in typeX),
        name='def_dist_total'
    )

    subProblem.addConstr(distTotal <= dadosProblema.maxDistancia, name='limite_max_dist')

    subProblem.addConstrs(
        (distTotal >= alpha * matrizCustos[i, i_prime] * (x[depot, i] + x[i_prime, depot] - 1)
         for i in R for i_prime in R if i != i_prime),
        name='sincronizacao_caminhao_drone'
    )

    subProblem._vars = x
    subProblem.addConstrs(
    (x[i, j] + x[j, i] <= 1 for i in nos for j in nos if i < j and (i, j) in typeX and (j, i) in typeX)
    )

    subProblem._vars = x
    subProblem._nos = nos
    subProblem._depot = depot

    subProblem.optimize(subtourelim)

    quantasAdicionadas = 0
    if subProblem.SolCount > 0 and subProblem.PoolObjVal < -1e-6:
        for k in range(subProblem.SolCount):
            subProblem.Params.SolutionNumber = k  
            if subProblem.PoolObjVal >= -1e-6:
                continue
            quantasAdicionadas += 1
            rota = []
            noAtual = depot
            custo = 0
            while(True):
                        for j in nos:
                            if noAtual != j and x[noAtual, j].Xn > 0.5:
                                custo += matrizCustos[noAtual, j]
                                noAtual = j
                                if(j != depot): rota.append(noAtual)
                                break
                        if(noAtual == depot): break
            
            droneRoutes.append({'nodes': rota, 'cost': custo})
            c.append(droneRoutes[len(R_d)]['cost'])
            for i in nos:
                delta[len(R_d), i] = int(i in rota)

            for i in R:
                start[len(R_d), i] = int(x[depot, i].Xn)
                finish[len(R_d), i] = int(x[i, depot].Xn)   
            R_d.append(len(R_d))
        
    return subProblem.Objval, len(R_d), quantasAdicionadas


def columnGeneration(dadosProblema):
    # Dados específicos
    R = dadosProblema.R
    V = dadosProblema.V
    custos = recalcuteDists(dadosProblema)
    nos = V + [dadosProblema.depot]

    # Criando o problema mestre
    problemaMestre, pi, sigma = criarMestre(dadosProblema)

    iteracao = 0
    total_colunas = 0

    while True:
        iteracao += 1

        problemaMestre.optimize()

        duaisPi = {i: pi[i].Pi for i in V}
        duaisSigma = {i: sigma[i].Pi for i in R}
        custoReduzido, novaRota, quantasRotas = resolverSubproblema(duaisPi, duaisSigma, dadosProblema, custos)

        if(custoReduzido >= -1e-6):
            print(f"Solução Ótima encontrada!")
            break
        total_colunas += quantasRotas
        print(f"Iteração {iteracao} | Colunas adicionadas {quantasRotas} | Colunas totais {total_colunas}")
        for q in range(1, quantasRotas+1):
            idx_col = novaRota - q
            coefs = [delta[idx_col, i] for i in V] + [start[idx_col, i] - finish[idx_col, i] for i in R] 
            constrs = [pi[piConstr] for piConstr in pi] + [sigma[sigmaConstr] for sigmaConstr in sigma]

            custoColuna = droneRoutes[idx_col]['cost']
            
            coluna = gp.Column(coefs, constrs)

            problemaMestre.addVar(
                lb = 0,
                obj = custoColuna,
                vtype = gp.GRB.CONTINUOUS,
                column = coluna,
                name=f'y[{idx_col}]'
            )
            #teste = problemaMestre.getConstrs()
            #vars = problemaMestre.getVars()
            #print(custoReduzido)
            #print(vars)
            #print(coefs)
            #print(constrs)
            #print(duaisPi)
            print(f"Rota {q} | Custo Relaxado: {problemaMestre.ObjVal:.2f} | Custo Reduzido: {custoReduzido:.2f} | Nova Coluna: {droneRoutes[idx_col]['nodes']}, cost: {droneRoutes[idx_col]['cost']:.2f}")

    print(f"Custo Relaxado: {problemaMestre.ObjVal:.2f}")

    print("===========================\nROTAS SELECIONADAS\n===========================")
    for r, rota in enumerate(droneRoutes):
            var = problemaMestre.getVarByName(f'y[{r}]')
            if var is not None and var.X > 1e-7:
                print(f'y[{r}]={var.X:.6f} | {rota}')
    print(f"Custo total relaxado: {problemaMestre.objVal}")



def subtourelim(model, where):
    if where == gp.GRB.Callback.MIPSOL:
        vals = model.cbGetSolution(model._vars)
        nos = model._nos
        depot = model._depot

        successor_map = {
            i: j for (i, j), val in vals.items() if val > 0.5
        }
        nodes = list(successor_map.keys())

        tour = subtour(successor_map, nodes)
        for t in tour:
            # qualquer ciclo que não contém o depot é obrigatoriamente um subtour
            if depot not in t:
                t_set = set(t)
                fora_de_t = [j for j in nos if j not in t_set]

                arcos_saida = gp.quicksum(
                    model._vars[i, j]
                    for i in t
                    for j in fora_de_t
                    if (i, j) in model._vars
                )

                for k in t:
                    visita_k = gp.quicksum(
                        model._vars[j, k]
                        for j in nos
                        if j != k and (j, k) in model._vars
                    )
                    model.cbLazy(arcos_saida >= visita_k)


def subtour(successor_map, nodes):
    unvisited = set(nodes)
    cycles = []
    
    while unvisited:
        start = unvisited.pop()
        thiscycle = [start]
        curr = successor_map.get(start)

        while curr in unvisited:
            unvisited.remove(curr)
            thiscycle.append(curr)
            curr = successor_map.get(curr)

        if curr == start:
            cycles.append(thiscycle)
            
    return cycles

def solveBinaryMasterProblem(dadosProblema):
    V = dadosProblema.V
    nos = V + [dadosProblema.depot]

    mestreInteiro = gp.Model("master_problem")
    # Adicionando variáveis
    # Escolha das rotas dos drones
    y = mestreInteiro.addVars(
        R_d,
        vtype=gp.GRB.BINARY,
        name="y",
    )

    mestreInteiro.Params.TimeLimit = 1800

    mestreInteiro.setObjective(
        gp.quicksum(
            c[r] * y[r]
            for r in R_d
        ),
        sense=gp.GRB.MINIMIZE
    )

    mestreInteiro.Params.OutputFlag = 0

    restrs = mestreInteiro.addConstrs(

        (
            gp.quicksum(delta[r, i] * y[r] for r in R_d)>= 1
            for i in V
        ),

        name="cover"
    )

    mestreInteiro.addConstrs(
            (
                gp.quicksum(y[r] * (start[r, i] - finish[r, i]) for r in R_d) == 0
                for i in dadosProblema.R
            )
    )

    mestreInteiro.optimize()
    print("==== ROTAS ESCOLHIDAS FINAIS ===")
    for r, rota in enumerate(droneRoutes):
        var = mestreInteiro.getVarByName(f'y[{r}]')
        if var is not None and var.X > 1e-7:
            print(f'y[{r}]={var.X:.6f} | {rota}')
    print(f"Custo total: {mestreInteiro.objVal}")