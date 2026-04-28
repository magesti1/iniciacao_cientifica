#include "funcoes.cpp"
#include <bits/stdc++.h>

using namespace std;

int main(int argc, char* argv[])
{
    string problemName = argv[1];
    int num_of_cities = 0;
    long double best_cost = 0;
    vector<vector<double>> distance_cost;
    vector<int> path;
    vector<pair<double, double>> coordinates;

    cout << problemName << endl;

    readFromFile(problemName, num_of_cities, coordinates, distance_cost, problemName);
    nearest_neighbor(num_of_cities, distance_cost, path, best_cost);

    if(argc > 1 && strcmp(argv[2], "1") == 0)
    {
        first_improvement_2opt(path, distance_cost, num_of_cities, best_cost);
    }
    else if(argc > 1 && strcmp(argv[2], "2") == 0)
    {
        best_improvement_2opt(path, distance_cost, num_of_cities, best_cost);
    }
    else if(argc > 1 && strcmp(argv[2], "3") == 0)
    {
        vector<int> points(num_of_cities);
        vector<bool> disponivel(num_of_cities, true);
        for(int i = 0; i<num_of_cities; i++) points[i] = i;
        vector<int> a;
        recursion(a, 0, best_cost, path, num_of_cities, distance_cost, disponivel, points);
    }

    //for(auto p : path) cout << p << " "; cout << endl;
    cout << "Best Cost: " << best_cost << endl;

    
    return 0;
}