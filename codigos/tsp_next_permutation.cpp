#include <bits/stdc++.h>
#define f first
#define s second

using namespace std;

void le_entrada(ifstream &fin, int &num_of_cities, vector<pair<int,int>> &coordinates, vector<vector<int>> &distance_cost, vector<int> &cities)
{
    fin >> num_of_cities;
    distance_cost.assign(num_of_cities, vector<int>(num_of_cities, 0));

    for(int i = 0; i<num_of_cities; i++) 
    {
        int x, y;
        fin >> x >> y;
        coordinates.push_back(pair<int,int>(x,y));
        cities.push_back(i);
    }

    for(int i = 0; i<num_of_cities; i++)
    {
        for(int j = 0; j<num_of_cities; j++)
            fin >> distance_cost[i][j];
    }
    
}

int main()
{
    ifstream fin("input.txt");
    int num_of_cities;
    vector<vector<int>> distance_cost;
    vector<pair<int, int>> coordinates;
    vector<int> cities;

    le_entrada(fin, num_of_cities, coordinates, distance_cost, cities);

       
    int best_cost = 10e5;
    do
    {
        int cost = 0;

        for(int i = 0; i<num_of_cities-1; i++)
        {
            cost += distance_cost[cities[i]][cities[i+1]];
            cout << cities[i] << " ";
        }
        cout << cities[num_of_cities-1] << "\n";

        cout << "Cost: " << cost << "\n";
        best_cost = min(cost, best_cost);

    } while (next_permutation(cities.begin(), cities.end()));
    
    cout << "Minimum cost: " << best_cost << endl;
    
    
    return 0;
}