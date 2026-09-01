import gurobi_model
import matplotlib.pyplot as plt

def cost(route, dadosProblema):
    soma = 0
    for i in range(0, len(route)-1):
         dist = dadosProblema.dists[(route[i], route[i+1])]
         soma += dist
    return soma

def two_opt(route, dadosProblema):
     best = route
     improved = True
     while improved:
          improved = False
          for i in range(1, len(route)-2):
               for j in range(i+1, len(route)):
                    if j-i == 1: continue # changes nothing, skip then
                    new_route = route[:]
                    new_route[i:j] = route[j-1:i-1:-1] # this is the 2woptSwap
                    if cost(new_route, dadosProblema) < cost(best, dadosProblema):  # what should cost be?
                         best = new_route
                         improved = True
          route = best
     return best

def aplicar_2opt_e_atualizar_sol(dados, varsX):
    # Dicionário de adjacência semelhante ao geraArquivoTexto
    arcos_ativos = {k: {} for k in dados.K}
    for (i, j, k), var in varsX.items():
        if var.X > 0.5:
            if i not in arcos_ativos[k]:
                arcos_ativos[k][i] = []
            arcos_ativos[k][i].append(j)

    novo_sol = "# Solucao Otimizada via 2-opt\n"

    # Reconstrução da rota para cada drone
    for k in dados.K:
        print(f"Rota {k}:")
        atual = dados.depot
        rota = [atual]
        
        for _ in range(dados.maxVoos + 5):
            if atual in arcos_ativos[k] and len(arcos_ativos[k][atual]) > 0:
                proximo = arcos_ativos[k][atual].pop(0)
                rota.append(proximo)
                atual = proximo
                if atual == dados.depot and len(rota) > 1:
                    break
            else:
                break

        print(rota)

        # Aplica a heurística se a rota tiver nós intermediários suficientes
        if len(rota) > 3:
            rota = two_opt(rota, dados) 

        print(rota)
        print("-----")

        # Converte a rota otimizada de volta para o formato de variável do Gurobi
        for idx in range(len(rota) - 1):
            i = rota[idx]
            j = rota[idx+1]
            novo_sol += f"x[{i},{j},{k}] 1\n"

    print("Escrito com a otimização 2-opt\n")

    # Sobrescreve o arquivo para a segunda leitura do solver
    with open("solucao_inicial.sol", "w", encoding='utf-8') as f:
        f.write(novo_sol)