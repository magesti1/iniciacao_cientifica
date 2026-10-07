def heuristica_zigue_zague(dadosProblema):
    """
    Gera a rota dos drones usando a lógica de zigue-zague a partir da rua central.
    Retorna uma lista de rotas, onde cada rota é uma lista de nós visitados.
    """
    y_center = dadosProblema.largura // 2
    
    # 1. Mapeamento da ESQUERDA da rua central
    left_points = []
    indo_para_esquerda = True
    for x in range(1, dadosProblema.altura - 1):
        if indo_para_esquerda:
            # Varre do centro em direção à borda esquerda
            for y in range(y_center - 1, 0, -1):
                left_points.append(x * dadosProblema.largura + y)
        else:
            # Varre da borda esquerda de volta para o centro
            for y in range(1, y_center):
                left_points.append(x * dadosProblema.largura + y)
        indo_para_esquerda = not indo_para_esquerda

    # 2. Mapeamento da DIREITA da rua central
    right_points = []
    indo_para_direita = True
    for x in range(1, dadosProblema.altura - 1):
        if indo_para_direita:
            # Varre do centro em direção à borda direita
            for y in range(y_center + 1, dadosProblema.largura - 1):
                right_points.append(x * dadosProblema.largura + y)
        else:
            # Varre da borda direita de volta para o centro
            for y in range(dadosProblema.largura - 2, y_center, -1):
                right_points.append(x * dadosProblema.largura + y)
        indo_para_direita = not indo_para_direita

    # 3. Concatenar todos os pontos do campo na ordem de visitação
    todos_pontos_campo = left_points + right_points

    # 4. Dividir em drones respeitando a capacidade maxVoos
    max_campo_por_rota = dadosProblema.maxVoos - 3 
    if max_campo_por_rota < 1:
        raise ValueError("maxVoos muito pequeno! Deve ser pelo menos 4.")

    rotas = []
    k_idx = 0
    for i in range(0, len(todos_pontos_campo), max_campo_por_rota):
        chunk = todos_pontos_campo[i : i + max_campo_por_rota]

        # Define decolagem e pouso no eixo Y central (onde o trator está)
        x_inicio = chunk[0] // dadosProblema.largura
        r_inicio = x_inicio * dadosProblema.largura + y_center
        
        x_fim = chunk[-1] // dadosProblema.largura
        r_fim = x_fim * dadosProblema.largura + y_center

        # Monta o ciclo completo
        rota = [dadosProblema.depot, r_inicio] + chunk + [r_fim, dadosProblema.depot]
        rotas.append(rota)
        k_idx += 1

        if k_idx >= dadosProblema.totalRotas and i + max_campo_por_rota < len(todos_pontos_campo):
            print("Aviso: totalRotas é insuficiente para cobrir o campo com a heurística.")
            break

    return rotas

def aplicar_MIP_Start(x, arcos, rotas, K):
    """
    Injeta a solução da heurística nas variáveis binárias do Gurobi
    """
    # Zera todo mundo inicialmente
    for (i, j, k) in arcos:
        x[i, j, k].Start = 0.0

    # Aplica valor 1.0 aos arcos utilizados pelas rotas da heurística
    for k_idx, rota in enumerate(rotas):
        if k_idx >= len(K):
            break
        for p in range(len(rota) - 1):
            i = rota[p]
            j = rota[p+1]
            if (i, j, k_idx) in x:
                x[i, j, k_idx].Start = 1.0