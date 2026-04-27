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
    string matrix_format;

    while(getline(fin, line))
    {
        auto pos = line.find(':') + 2;
        auto tam = line.size();
        if(line.substr(0, 4) == "NAME")                     problemName = line.substr(pos, tam-pos);
        else if(line.substr(0, 16) == "EDGE_WEIGHT_TYPE")   distance_type = line.substr(pos, tam-pos);
        else if(line.substr(0, 18) == "EDGE_WEIGHT_FORMAT") matrix_format = line.substr(pos, tam-pos);

        else if(line.substr(0, 9) == "DIMENSION")
        {
            n = stoi(line.substr(pos, tam-pos));
            coordinates.resize(n);
            distance_cost.resize(n, vector<int>(n));
        }

        else if(line.substr(0, 18) == "NODE_COORD_SECTION" || line.substr(0, 18) == "DISPLAY_DATA_SECTION")
        {
            int aux;
            for(int i = 0; i<n; i++) fin >> aux >> coordinates[i].first >> coordinates[i].second;
        }

        else if(line.substr(0, 19) == "EDGE_WEIGHT_SECTION")
        {
            int rowStart, colStart;
            if(matrix_format == "UPPER_ROW ")            readUpperRowMatrix(fin, distance_cost, n);
            else if(matrix_format == "FULL_MATRIX ")     readFullMatrix(fin, distance_cost, n);
            else if(matrix_format == "UPPER_DIAG_ROW ")  readUpperDiagRow(fin, distance_cost, n);
            else if(matrix_format == "LOWER_DIAG_ROW ")  readLowerDiagRow(fin, distance_cost, n);
        }
    }
    
    if(distance_type == "EUC_2D") EUC2D(coordinates, distance_cost);

    cout << distance_type << endl << matrix_format << endl;
    
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

    readFromFile("brazil58.tsp", num_of_cities, coords, matrix, problemName);
    return 0;
}