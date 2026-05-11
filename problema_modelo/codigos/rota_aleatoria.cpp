#include <bits/stdc++.h>

using namespace std;

int main()
{
    int x, y;
    cin >> x >> y;
    
    vector<vector<bool>> grid(y, vector<bool>(x, 0));

    vector<pair<int, int>> path;

    grid[0][0] = true;
    path.push_back(pair<int,int>(0,0));

    vector<pair<int, int>> sortear = { {-1, 0}, {1, 0}, {0, -1}, {0, 1}};

    int cont = 1;

    pair<int,int> last = {0,0};
    while(true)
    {
        
        pair<int, int> sorted;
        while(true)
        {
            int idx_sorteado = rand()%4;
            sorted = {last.first + sortear[idx_sorteado].first, last.second + sortear[idx_sorteado].second};
            if(sorted.first >= 0 && sorted.first < x && sorted.second >= 0 && sorted.second < y) break;
        }

        if(!grid[last.first][last.second])
        {
            grid[last.first][last.second] = 1;
            cont++;
        }
        last = sorted;
        if(cont >= x*y) break;
        path.push_back(last);
    }

    for(auto p : path) cout << p.first << " " << p.second << "\n";
    cout << cont << "\n";


    return 0;
}