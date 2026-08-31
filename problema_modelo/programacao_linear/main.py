import os
import gurobi_model

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

        # Criando os dados do problema
        dados = gurobi_model.dadosDoProblema(
            x,      # larguraGrid
            y,      # alturaGrid
            k,      # totalRotas
            m,      # maxVoos
            True,   # firstSolution
            1.0,    # alpha
            1      # T_c
        )
        
        # Rodando o modelo de programação linear pela primeira vez, para o MIP Start
        modelo, varsX = gurobi_model.runModel(dados)

        # Setando a variável firstSolution para False, para então gerar a solução definitiva
        dados.firstSolution = False
        modelo, varsX = gurobi_model.runModel(dados)

        # Criando a pasta em que ficará salva a solução da instância
        experimentFolderName = f'{x}_{y}_{k}_{m}_solution'
        folderName = os.path.join("solutionsCarregamento", experimentFolderName) 

        os.makedirs(folderName, exist_ok=True)

        # Gerando o arquivo texto e imagem e os salvando na pasta da instância
        gurobi_model.geraArquivoTexto(dados, modelo, folderName, varsX)

        gurobi_model.geraImagemRotas(dados, modelo, folderName, varsX)
            

if __name__ == "__main__":
    main()