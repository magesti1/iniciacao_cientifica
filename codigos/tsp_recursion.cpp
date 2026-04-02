#include <bits/stdc++.h>
#define f first
#define s second

using namespace std;

int num_of_cities = 8;
vector<int> points;
vector<bool> disponivel;
vector<vector<int>> distance_cost; // set all cost to 0
vector<pair<int,int>> coordinates;

void le_entrada(ifstream &fin)
{
    fin >> num_of_cities;
    distance_cost.assign(num_of_cities, vector<int>(num_of_cities, 0));
    disponivel.assign(num_of_cities, true);
    
    for(int i = 0; i<num_of_cities; i++) 
    {
        int x, y;
        fin >> x >> y;
        coordinates.push_back(pair<int,int>(x,y));
        points.push_back(i);
    }
    
    for(int i = 0; i<num_of_cities; i++)
    {
        for(int j = 0; j<num_of_cities; j++)
            fin >> distance_cost[i][j];
    }
    
}

void search(vector<int> cities, int cost, int &best_cost, vector<int> &best_path)
{
    if(cities.size() == num_of_cities)
    {
        cost += distance_cost[cities[num_of_cities-1]][cities[0]];
        cities.push_back(cities[0]);
        for(auto c : cities)
                cout << c << " ";
            cout << endl << cost;
        cout << endl;
        if(cost < best_cost)
        {
            best_cost = cost;
            best_path.clear();
            for(auto c : cities) best_path.push_back(c);
        }
        return;
    }

    if(cost > best_cost) return; // caso já esteja mais caro que o melhor, nem tenta mais

    for(int i = 0; i<num_of_cities; i++)
    {       
        if(disponivel[i])
        {
            cities.push_back(points[i]);
            disponivel[i] = false;
            if(cities.size() == 1)
                search(cities, cost, best_cost, best_path);
            else
                search(cities, cost + distance_cost[cities[cities.size()-2]][cities[cities.size()-1]], best_cost, best_path);
            disponivel[i] = true;
            cities.pop_back();
        }
    }

}

int main()
{

    ifstream fin("input.txt");
    le_entrada(fin);
    
    vector<int> a;
    int best_cost = 10e8;
    vector<int> best_path;
    search(a, 0, best_cost, best_path);
    for(auto c : best_path) cout << c << " ";
    cout << endl;
    cout << "Minimum cost: " << best_cost << endl;



    return 0;
}