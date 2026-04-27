#include "auxiliares.cpp"
#include <bits/stdc++.h>

using namespace std;

void readFromFile(string fileName, int &n, vector<pair<int, int>> &coordinates, vector<vector<int>> &distance_cost, string &problemName)
{
    ifstream fin("/home/eduardo/Documentos/iniciacao_cientifica/iniciacao_cientifica/tsplib/" + fileName);
    string line;
    if(!fin.is_open())
    {
        cout << "file is not open.\n";
        return;
    }

    string distance_type;

    while(getline(fin, line))
    {
        auto pos = line.find(':') + 2;
        auto tam = line.size();
        if(line.substr(0, 4) == "NAME") 
        {   
            problemName = line.substr(pos, tam-pos);
        }
        else if(line.substr(0, 9) == "DIMENSION")
        {
            n = stoi(line.substr(pos, tam-pos));
            coordinates.resize(n);
            distance_cost.resize(n, vector<int>(n));
        }
        else if(line.substr(0, 16) == "EDGE_WEIGHT_TYPE")
        {
            distance_type = line.substr(pos, tam-pos);
        }
        else if(line.substr(0, 18) == "NODE_COORD_SECTION")
        {
            int aux;
            int i = 0;
            while(fin >> aux)
            {
                fin >> coordinates[i].first >> coordinates[i].second;
                i++;
            }
        }
    }
    
    if(distance_type == "EUC_2D") EUC2D(coordinates, distance_cost);

    for(auto row : distance_cost)
    {
        for(auto col : row) cout << col << " ";
        cout << endl;
    }

}

int main()
{
    string problemName;
    int num_of_cities = 0;
    vector<pair<int, int>> coords;
    vector<vector<int>> matrix;

    readFromFile("a280.tsp", num_of_cities, coords, matrix, problemName);
    return 0;
}