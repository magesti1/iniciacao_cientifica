import os
from gurobi_model import run_optmization

def main():
    print('---Starting Execution---')
    print('Largura Altura Número máximo de rotas Distância máxima de cada rota')
    print('x y k m')
    while True:
        linha = input().split()
        x = int(linha[0])
        y = int(linha[1])
        if(x == 0 and y == 0): 
            break
        k = int(linha[2])
        m = int(linha[3])

        if os.path.exists("solucao_inicial.mst"):
            os.remove("solucao_inicial.mst")
    
        run_optmization(x,y,k,m, True)
        run_optmization(x, y, k, m, False)
    

if __name__ == "__main__":
    main()