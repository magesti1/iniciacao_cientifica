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

void nearest_neighbor(int n, vector<vector<int>> &distance_cost, vector<int> &path, int &total_cost)
{
    srand(time(0));
    vector<bool> visited(n, false);
    total_cost = 0;
    int current_city = rand()%n;

    while(true)
    {
        path.push_back(current_city);
        visited[current_city] = true;
        int min_dist = INT_MAX; //pode falhar para distâncias muito grandes
        int next_city = -1;
        bool inicio = false;

        for(int i = 0; i<n; i++)
        {
            if(visited[i]) continue;
            if(distance_cost[current_city][i] < min_dist)
            {
                min_dist = distance_cost[current_city][i];
                next_city = i;
                inicio = false;
            }
            if(distance_cost[i][path[0]] < min_dist)
            {
                min_dist = distance_cost[i][path[0]];
                next_city = i;
                inicio = true;
            }
        }

        //cout << inicio << endl;
        //for(auto p : path) cout << p << " "; cout << endl;
        if(inicio) reverse(path.begin(), path.end());

        //for(auto p : path) cout << p << " "; cout << endl;
        //cout << path[path.size()-1] << " " << next_city << " " << distance_cost[path[path.size()-1]][next_city] << " " << min_dist << endl;

        if(next_city == -1) 
        {
            total_cost += distance_cost[path[n-1]][path[0]];
            path.push_back(path[0]);   
            break;
        }
        else current_city = next_city;
        total_cost += min_dist;
    }

    for(auto p : path) cout << p << " ";
    cout << endl;
    cout << "Cost: " << total_cost << endl;

}

void search(vector<int> cities, int cost, int &best_cost, vector<int> &best_path)
{
    if(cities.size() == num_of_cities)
    {
        cost += distance_cost[cities[num_of_cities-1]][cities[0]];
        cities.push_back(cities[0]);
        //for(auto c : cities)
        //        cout << c << " ";
        //    cout << endl << cost;
        //cout << endl;
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
    int best_cost = 0;
    vector<int> best_path;

    nearest_neighbor(num_of_cities, distance_cost, best_path, best_cost);

    //search(a, 0, best_cost, best_path);
    
    int custo = 0;
    for(int i = 0; i<num_of_cities-1; i++) 
    {
        custo += distance_cost[best_path[i]][best_path[i+1]];
        cout << best_path[i] << " " << best_path[i+1] << " " << distance_cost[best_path[i]][best_path[i+1]] << endl;
    }
    custo += distance_cost[best_path[0]][best_path[num_of_cities-1]];
    
    for(auto c : best_path) cout << c << " ";
    cout << endl;
    cout << "Real cost: " << custo << endl;

    return 0;
}