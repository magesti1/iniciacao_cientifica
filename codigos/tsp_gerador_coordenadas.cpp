#include <bits/stdc++.h>
#define f first
#define s second

using namespace std;

void popula_cordenadas(int n, vector<pair<int,int>> &coordinates, int minx, int maxx, int miny, int maxy)
{
    for(int i = 0; i<n; i++) //população das coordenadas
    {
        int r1 = minx + rand()%(maxx-minx+1);
        int r2 = miny + rand()%(maxy-miny+1);
        coordinates.push_back(pair<int,int>(r1, r2));
    }
}

void popula_distancia(vector<pair<int,int>> &coordinates, int n, vector<vector<int>> &distance_cost)
{
    for(int i = 0; i<n; i++)
    {
        for(int j = i + 1; j<n; j++)
        {
            int dist = ((coordinates[i].f - coordinates[j].f) * (coordinates[i].f - coordinates[j].f)) +
            ((coordinates[i].s - coordinates[j].s) * (coordinates[i].s - coordinates[j].s));
            distance_cost[i][j] = dist;
            distance_cost[j][i] = dist;
        }
    }
}

void imprime_saida(ofstream &fout, int num_of_cities, vector<pair<int,int>> &coordinates, vector<vector<int>> &distance_cost)
{
    fout << num_of_cities << endl;

    for(int i = 0; i<num_of_cities; i++)
    {
        fout << coordinates[i].f << " " << coordinates[i].s << endl;
    }

    for(int i = 0; i<num_of_cities; i++)
    {
        for(int j = 0; j<num_of_cities; j++)
            fout << distance_cost[i][j] << " ";
        fout << endl;
    }
}

int main(int argc, char ** argv) // 1: first improvement, 2: best improvement
{   
    srand(time(0));
    ofstream fout("input.txt");

    
    int num_of_cities, minx, maxx, miny, maxy;

    cout << "Ordem da entrada: \nnúmero de cidades\nminX\nmaxX\nminY\nmaxY\n";

    cin >> num_of_cities >> minx >> maxx >> miny >> maxy;


    vector<pair<int,int>> coordinates;
    vector<vector<int>> distance_cost(num_of_cities, vector<int>(num_of_cities, 0));

    popula_cordenadas(num_of_cities, coordinates, minx, maxx, miny, maxy);

    popula_distancia(coordinates, num_of_cities, distance_cost);

    imprime_saida(fout, num_of_cities, coordinates, distance_cost);

    fout.close();
    return 0;
}