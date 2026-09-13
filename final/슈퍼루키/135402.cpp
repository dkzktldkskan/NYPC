#include <bits/stdc++.h>
using namespace std;

struct Motion {
    int y=0,x=0;
    int vx=0,vy=0;               // state AFTER the previous frame
    bool prevJumpTime=false;     // jump-time at the start of previous frame
    bool prevJumped=false;       // whether J actually happened in previous frame
    int hold=0;                  // remaining jump-hold frames (0..2)
};

struct Action {
    int dir=0;                   // -1,0,+1
    bool J=false;
};

struct SearchNode {
    Motion s;
    int usedHaz=0;               // number of spike objects crossed in this planned path
    int parent=-1;
    Action act;
    int g=0;                     // weighted planning cost (frames + resource/risk penalty)
    bool hitTarget=false;        // target may have been swept through mid-frame
};

int main(){
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    int N,M; string type;
    if(!(cin>>N>>M>>type)) return 0;
    const int H=6*N, W=6*M, K=N*M;

    const bool isChoice      = (type=="Choice");
    const bool isJump        = (type=="Jump");
    const bool isPits        = (type=="Pits");
    const bool isPuzzle      = (type=="Puzzle");
    const bool isMaze        = (type=="Maze");
    // Handcrafted deliberately has no special structural assumptions.
    const bool lightFSM      = isPuzzle;

    vector<string> world(H,string(W,'?'));
    vector<unsigned char> cellSeen(K,0);
    vector<signed char> coinAlive(K,-1);   // -1 unknown, 0 no/gone, 1 known alive
    vector<signed char> shieldAlive(K,-1); // same, for '*'
    vector<int> badTargetUntil(K,0);

    auto insideCell=[&](int r,int c){ return 0<=r&&r<N&&0<=c&&c<M; };
    auto cid=[&](int r,int c){ return r*M+c; };
    auto insidePix=[&](int y,int x){ return 0<=y&&y<H&&0<=x&&x<W; };

    auto decode=[&](const string& enc){
        string s; s.reserve(2304);
        for(size_t i=0;i<enc.size();){
            char ch=enc[i++];
            int cnt=0;
            while(i<enc.size() && isdigit((unsigned char)enc[i])) cnt=cnt*10+(enc[i++]-'0');
            s.append(cnt,ch);
        }
        vector<string> a(48,string(48,'.'));
        if(s.size()==2304){
            for(int r=0;r<48;r++) for(int c=0;c<48;c++) a[r][c]=s[r*48+c];
        }
        return a;
    };

    auto emit=[&](Action a){
        int n=(a.dir!=0)+(a.J?1:0);
        cout<<n<<'\n';
        if(a.dir<0) cout<<"<\n";
        if(a.dir>0) cout<<">\n";
        if(a.J) cout<<"J\n";
        cout.flush();
    };

    // Prefix sums rebuilt only when a new plan is needed.
    const int PW=W+1;
    vector<int> prefBlock((H+1)*(W+1));      // known solid only: # or O
    vector<int> prefUnknown((H+1)*(W+1));
    vector<int> prefBox((H+1)*(W+1));
    vector<int> prefHaz((H+1)*(W+1));        // spike pixels
    vector<int> prefJump((H+1)*(W+1));

    auto buildPrefix=[&](){
        fill(prefBlock.begin(),prefBlock.end(),0);
        fill(prefUnknown.begin(),prefUnknown.end(),0);
        fill(prefBox.begin(),prefBox.end(),0);
        fill(prefHaz.begin(),prefHaz.end(),0);
        fill(prefJump.begin(),prefJump.end(),0);

        for(int y=0;y<H;y++){
            int rowBase=(y+1)*PW;
            int prevBase=y*PW;
            for(int x=0;x<W;x++){
                char ch=world[y][x];
                int b=(ch=='#'||ch=='O');
                int un=(ch=='?');
                int j=(ch=='+');
                int p=rowBase+x+1;
                prefBlock[p]=b+prefBlock[prevBase+x+1]+prefBlock[rowBase+x]-prefBlock[prevBase+x];
                prefUnknown[p]=un+prefUnknown[prevBase+x+1]+prefUnknown[rowBase+x]-prefUnknown[prevBase+x];
                prefJump[p]=j+prefJump[prevBase+x+1]+prefJump[rowBase+x]-prefJump[prevBase+x];
                // Jump maps contain no box/spike decision mechanics.  Do not spend two
                // extra full-map prefix calculations there; this leaves navigation behavior
                // unchanged while cutting a large part of the max-grid planning overhead.
                if(!isJump){
                    int o=(ch=='O');
                    int hz=(ch=='^');
                    prefBox[p]=o+prefBox[prevBase+x+1]+prefBox[rowBase+x]-prefBox[prevBase+x];
                    prefHaz[p]=hz+prefHaz[prevBase+x+1]+prefHaz[rowBase+x]-prefHaz[prevBase+x];
                }
            }
        }
    };

    auto rectSum=[&](const vector<int>& p,int y1,int x1,int y2,int x2)->int{
        if(y1>y2||x1>x2) return 0;
        if(y1<0||x1<0||y2>=H||x2>=W) return 1e9;
        int A=y1*PW+x1, B=y1*PW+(x2+1), C=(y2+1)*PW+x1, D=(y2+1)*PW+(x2+1);
        return p[D]-p[B]-p[C]+p[A];
    };

    auto blockedRect=[&](int y,int x){ return rectSum(prefBlock,y,x,y+5,x+5)>0 || rectSum(prefUnknown,y,x,y+5,x+5)>0; };
    auto hazardRect=[&](int y,int x){ return rectSum(prefHaz,y,x,y+5,x+5)>0; };
    auto jumpRect=[&](int y,int x){ return rectSum(prefJump,y,x,y+5,x+5)>0; };

    // Coin/shield items occupy the same 4x4 middle pattern inside a logical cell.
    auto itemOverlap=[&](int y,int x,int z)->bool{
        if(z<0) return false;
        int r=z/M, c=z%M;
        static const int shape[4][4] = {
            {0,1,1,0},
            {1,1,1,1},
            {1,1,1,1},
            {0,1,1,0}
        };
        for(int dy=0;dy<4;dy++) for(int dx=0;dx<4;dx++) if(shape[dy][dx]){
            int py=6*r+1+dy, px=6*c+1+dx;
            if(y<=py && py<=y+5 && x<=px && px<=x+5) return true;
        }
        return false;
    };
    auto bottomSolid=[&](int y,int x){
        if(y+6>=H) return true;
        return rectSum(prefBlock,y+6,x,y+6,x+5)>0;
    };
    auto boxSide=[&](int y,int x,int dir){
        int xx=(dir<0?x-1:x+6);
        if(xx<0||xx>=W) return false;
        return rectSum(prefBox,y,xx,y+5,xx)>0;
    };
    auto unknownSide=[&](int y,int x,int dir){
        int xx=(dir<0?x-1:x+6);
        if(xx<0||xx>=W) return true;
        return rectSum(prefUnknown,y,xx,y+5,xx)>0;
    };
    auto solidSide=[&](int y,int x,int dir){
        int xx=(dir<0?x-1:x+6);
        if(xx<0||xx>=W) return true;
        return rectSum(prefBlock,y,xx,y+5,xx)>0;
    };

    // Conservative one-pixel pushability check on the currently observed box layout.
    // Exact box physics is still not part of A*, but this lets the controller prefer
    // opening a box route over spending a shield / taking spike damage.
    auto canPushSide=[&](int y,int x,int dir){
        int sx=(dir<0?x-1:x+6);
        if(sx<0||sx>=W) return false;
        bool any=false;
        for(int yy=y;yy<=y+5;yy++){
            if(!insidePix(yy,sx) || world[yy][sx]!='O') continue;
            any=true;
            int xx=sx;
            while(0<=xx && xx<W && world[yy][xx]=='O') xx+=dir;
            if(xx<0||xx>=W) return false;
            char dst=world[yy][xx];
            if(dst=='#' || dst=='?') return false;
        }
        return any;
    };
    auto viewUnknownCount=[&](int y,int x){
        int y1=max(0,y-21), x1=max(0,x-21);
        int y2=min(H-1,y+26), x2=min(W-1,x+26);
        return rectSum(prefUnknown,y1,x1,y2,x2);
    };

    // Exact player transition on the currently known STATIC collision map.
    // Boxes are intentionally treated as solid/unpushable in this planner.
    // Spikes do NOT block motion. We count entries into spike regions so the search
    // can spend shields first and, if necessary, accept an unshielded hit.
    auto stepState=[&](const Motion& in, Action a, Motion& out,
                       int targetZ, bool &hitTarget, int &hazEntries)->bool{
        Motion s=in;
        hitTarget=itemOverlap(s.y,s.x,targetZ);
        hazEntries=0;
        bool wasHaz=hazardRect(s.y,s.x);

        auto touched=[&](){
            if(itemOverlap(s.y,s.x,targetZ)) hitTarget=true;
            bool nowHaz=hazardRect(s.y,s.x);
            if(nowHaz && !wasHaz) hazEntries++;
            wasHaz=nowHaz;
        };

        bool jt = bottomSolid(s.y,s.x) || jumpRect(s.y,s.x);
        bool jumped=false;
        int nextHold=0;
        if(a.J){
            bool fresh = jt || (s.prevJumpTime && !s.prevJumped);
            if(fresh){
                jumped=true;
                nextHold=2;
            }else if(s.prevJumped && s.hold>0){
                jumped=true;
                nextHold=s.hold-1;
            }
        }

        if(jumped) s.vy=-3;

        // vertical move, pixel by pixel
        if(s.vy<0){
            int cnt=-s.vy;
            for(int k=0;k<cnt;k++){
                int ny=s.y-1;
                if(ny<0) { s.vy=0; break; }
                if(rectSum(prefUnknown,ny,s.x,ny,s.x+5)>0) return false;
                if(rectSum(prefBlock,ny,s.x,ny,s.x+5)>0){ s.vy=0; break; }
                s.y=ny;
                touched();
            }
        }else if(s.vy>0){
            int cnt=s.vy;
            for(int k=0;k<cnt;k++){
                int ny=s.y+1;
                if(ny+5>=H) { s.vy=0; break; }
                if(rectSum(prefUnknown,ny+5,s.x,ny+5,s.x+5)>0) return false;
                if(rectSum(prefBlock,ny+5,s.x,ny+5,s.x+5)>0){ s.vy=0; break; }
                s.y=ny;
                touched();
            }
        }

        if(!bottomSolid(s.y,s.x) && !jumped) s.vy=min(6,s.vy+1);

        // horizontal deceleration
        if(s.vx!=0 && (a.dir==0 || (s.vx>0?a.dir<0:a.dir>0))){
            s.vx += (s.vx>0?-1:1);
        }

        if(a.dir!=0){
            // Box pushing is still left for the dedicated puzzle layer.
            if(boxSide(s.y,s.x,a.dir)) return false;
            if(unknownSide(s.y,s.x,a.dir)) return false;
            if(!solidSide(s.y,s.x,a.dir)){
                s.vx=max(-4,min(4,s.vx+a.dir));
            }
        }

        if(s.vx<0){
            int cnt=-s.vx;
            for(int k=0;k<cnt;k++){
                int nx=s.x-1;
                if(nx<0){ s.vx=0; break; }
                if(rectSum(prefUnknown,s.y,nx,s.y+5,nx)>0) return false;
                if(rectSum(prefBlock,s.y,nx,s.y+5,nx)>0){ s.vx=0; break; }
                s.x=nx;
                touched();
            }
        }else if(s.vx>0){
            int cnt=s.vx;
            for(int k=0;k<cnt;k++){
                int nx=s.x+1;
                if(nx+5>=W){ s.vx=0; break; }
                if(rectSum(prefUnknown,s.y,nx+5,s.y+5,nx+5)>0) return false;
                if(rectSum(prefBlock,s.y,nx+5,s.y+5,nx+5)>0){ s.vx=0; break; }
                s.x=nx;
                touched();
            }
        }

        s.prevJumpTime=jt;
        s.prevJumped=jumped;
        s.hold=jumped?nextHold:0;
        out=s;
        return true;
    };

    auto pack=[&](const Motion& s)->uint64_t{
        uint64_t q=(uint64_t)(s.y*W+s.x);
        q=q*9+(s.vx+4);
        q=q*10+(s.vy+3);
        q=q*2+(s.prevJumpTime?1:0);
        q=q*2+(s.prevJumped?1:0);
        q=q*3+s.hold;
        return q;
    };

    deque<Action> plan;
    int plannedTarget=-1;
    char plannedKind=0;          // '$' or '*'
    Motion motion;
    bool haveMotion=false;
    Motion predicted;
    bool havePrediction=false;
    int prevActualY=-1, prevActualX=-1;
    int mismatchStreak=0;
    int noProgress=0;
    int retryAt=0;
    int seenCells=0;
    int failedSeenCells=-1000000;
    int xStall=0, fallbackDir=1, fallbackLock=0;
    int prevFallbackX=-1;
    vector<int> cellVisits(K,0), visitEpoch(K,0), badFrontierUntil(K,0);
    int currentVisitEpoch=1;
    auto getVisit=[&](int z)->int{
        return visitEpoch[z]==currentVisitEpoch ? cellVisits[z] : 0;
    };
    auto addVisit=[&](int z){
        if(visitEpoch[z]!=currentVisitEpoch){
            visitEpoch[z]=currentVisitEpoch;
            cellVisits[z]=0;
        }
        cellVisits[z]++;
    };
    int fallbackGoal=-1, fallbackGoalSince=0, fallbackGoalSeenCells=0;
    int lastDiscoveryT=0, escapeUntil=-1, escapeDir=1;
    bool coarseGuidance=false;
    int coarseNextR=-1, coarseNextC=-1;
    int boxCommitTarget=-1;
    char boxCommitKind=0;
    int boxCommitUntil=-1;
    int choiceColumnLock=-1;  // legacy Choice planner lock (dedicated policy below owns real choices)
    int choiceDropCol=-1;       // dedicated Choice: chosen vertical chute column
    int choiceDropStartY=-1;
    int choiceDropSince=-1;
    bool choiceDropStarted=false;
    bool choiceDropHasCoin=false;   // selected chute was chosen because it contains a mandatory coin
    bool choiceDropCoinCollected=false;
    // Leave enough CPU headroom for 10,000-frame map decoding / fallback control.
    const double PLAN_CPU_BUDGET = lightFSM ? 0.0
        : (isJump ? (K>=6000?0.58:(K>=3000?0.84:1.08))
        : (isChoice ? 0.72
        : (isPits ? (K>=6000?0.62:0.82)
        : (isMaze ? (K>=6000?0.78:0.98)
        : (K>=6000 ? 0.72 : (K>=3000 ? 0.90 : 1.08))))));

    // Cheap observed-state guard against infinite in-place jumps under a ceiling / false up-edge.
    Action lastIssued{0,false};
    bool haveLastIssued=false;
    int failedJumpStreak=0;
    int jumpBanUntil=-1;
    int jumpEscapeDir=1;

    // Lightweight per-generator policies.  These intentionally avoid global A* on the
    // highly regular Puzzle generator.  Jump deliberately keeps the proven generic mover.
    int puzzleDir=-1;
    int puzzleLevelY=-1;

    // Puzzle-specific state.  The generator contains optional one-way coin detours
    // on the left side of a sweep.  Once such a coin is seen we lock onto it until
    // it is collected (or the lock times out), instead of blindly switching rows.
    int puzzleCoinTarget=-1;
    int puzzleCoinSince=-1;

    // A Puzzle coin lane is a detour: go left to collect the coin, then explicitly
    // return to the main route instead of continuing into the dead-end wall.
    bool puzzleReturningCoin=false;
    int puzzleReturnDir=0;

    // Only a jump that STARTED while overlapping a jump block may change Puzzle
    // floors.  Ordinary spike jumps must never flip the snake direction.
    bool puzzleLiftActive=false;
    int puzzleLiftStartY=-1;
    bool puzzleLiftWasReturn=false;
    // A mid-elevator left branch is a detour, not a floor transition.
    // Keep it separate so landing in the branch does not flip puzzleDir.
    bool puzzleLiftDetourExit=false;

    // Puzzle starts on the long right-hand jump-block elevator.  Coin lanes branch
    // left from this shaft and are one-way detours: enter, take the coin, then return
    // to the SAME shaft and continue climbing.  The old code looked for the coin
    // itself from the shaft, which fails whenever the coin is farther than 24 pixels.
    // Instead we recognize the generator's safe flat left branch locally and explore
    // that branch.  Total work stays linear in the revealed corridors (N*M <= 10000).
    bool puzzleInitialAscent=true;
    int puzzleShaftX=-1;
    int puzzleInitialDetour=0; // 0=on shaft, 1=going left, 2=returning right
    int puzzleInitialDetourFrames=0;
    int puzzleInitialDetourStartX=-1;
    int puzzleInitialDetourRow=-1;
    bool puzzleInitialDetourEntered=false;
    int puzzleInitialDetourLastX=-1;
    int puzzleInitialDetourStall=0;
    vector<unsigned char> puzzleInitialBranchDone(N,0);

    // Every encountered box is a required operation in Puzzle: touching/pushing it
    // at least once is more important than treating it as a generic obstacle.
    // The key is the logical cell in which the box was first deliberately pushed.
    vector<unsigned char> puzzleBoxTouched(K,0);

    // Puzzle far-left transition: after a horizontal sweep reaches its left dead-end,
    // a mandatory coin lane may be directly below.  Once selected this is a HARD lock:
    // no return/box/elevator FSM is allowed to reverse direction before the fall finishes.
    int puzzleDropTarget=-1;
    int puzzleDropX=-1;
    int puzzleDropStartY=-1;
    int puzzleDropSince=-1;
    bool puzzleDropStarted=false;
    bool puzzleDropCollected=false;

    auto isCellHard=[&](int r,int c){
        if(!insideCell(r,c) || !cellSeen[cid(r,c)]) return true;
        int cy=6*r+3, cx=6*c+3;
        char ch=world[cy][cx];
        return ch=='#' || ch=='O';
    };

    auto cellTrapPenalty=[&](int r,int c,int startR){
        if(!insideCell(r,c) || !cellSeen[cid(r,c)]) return 1000000;
        char here=world[6*r+3][6*c+3];
        if(here=='#') return 1000000;

        static const int dr[4]={-1,1,0,0};
        static const int dc[4]={0,0,-1,1};
        int deg=0;
        for(int k=0;k<4;k++){
            int nr=r+dr[k],nc=c+dc[k];
            if(!insideCell(nr,nc) || !cellSeen[cid(nr,nc)]) continue;
            char ch=world[6*nr+3][6*nc+3];
            if(ch!='#') deg++;
        }

        int risk=0;
        if(deg<=1) risk+=120;
        else if(deg==2) risk+=15;

        int drop=max(0,r-startR);
        bool plusNear=false;
        for(int rr=max(0,r-3);rr<=min(N-1,r+3)&&!plusNear;rr++)
            for(int cc=max(0,c-2);cc<=min(M-1,c+2);cc++)
                if(cellSeen[cid(rr,cc)] && world[6*rr+3][6*cc+3]=='+'){
                    plusNear=true;
                    break;
                }
        if(drop>0 && !plusNear) risk+=55*drop;
        return risk;
    };

    // Exact A* to one concrete item, or to a viewpoint that exposes unknown map.
    auto searchPlan=[&](const Motion& start, int targetZ,
                        int shieldStart, bool wantFrontier, int maxHazEntries=32)->deque<Action>{
        if(blockedRect(start.y,start.x)) return {};

        const int INF=1e9;
        vector<int> hcell(K,INF);
        queue<int> cq;

        if(!wantFrontier){
            if(targetZ<0) return {};
            hcell[targetZ]=0;
            cq.push(targetZ);
        }else{
            for(int r=0;r<N;r++) for(int c=0;c<M;c++){
                int z=cid(r,c);
                if(!cellSeen[z] || isCellHard(r,c)) continue;
                bool nearUnknown=false;
                static const int dr4[4]={-1,1,0,0};
                static const int dc4[4]={0,0,-1,1};
                for(int k=0;k<4;k++){
                    int nr=r+dr4[k],nc=c+dc4[k];
                    if(insideCell(nr,nc) && !cellSeen[cid(nr,nc)]) nearUnknown=true;
                }
                if(nearUnknown){
                    hcell[z]=0;
                    cq.push(z);
                }
            }
        }

        // Unlike v1.3, the heuristic does not walk straight through known walls/boxes.
        static const int dr4[4]={-1,1,0,0};
        static const int dc4[4]={0,0,-1,1};
        while(!cq.empty()){
            int v=cq.front(); cq.pop();
            int r=v/M,c=v%M;
            for(int k=0;k<4;k++){
                int nr=r+dr4[k],nc=c+dc4[k];
                if(!insideCell(nr,nc) || isCellHard(nr,nc)) continue;
                int nv=cid(nr,nc);
                if(hcell[nv]>hcell[v]+1){
                    hcell[nv]=hcell[v]+1;
                    cq.push(nv);
                }
            }
        }

        auto heur=[&](const Motion& a){
            int r=clamp((a.y+3)/6,0,N-1);
            int c=clamp((a.x+3)/6,0,M-1);
            int h=hcell[cid(r,c)];
            return h>=INF/2?0:h;
        };

        struct QN{ int f,g,idx; };
        struct Cmp{ bool operator()(const QN&a,const QN&b)const{
            if(a.f!=b.f) return a.f>b.f;
            return a.g>b.g;
        }};
        priority_queue<QN,vector<QN>,Cmp> pq;
        vector<SearchNode> nodes; nodes.reserve(70000);
        unordered_map<uint64_t,int> bestG; bestG.reserve(90000);

        const int HAZCAP=32;
        auto searchKey=[&](const Motion& m,int used){
            return pack(m)*(uint64_t)(HAZCAP+1) + (uint64_t)min(used,HAZCAP);
        };

        nodes.push_back({start,0,-1,{0,false},0,itemOverlap(start.y,start.x,targetZ)});
        bestG[searchKey(start,0)]=0;
        pq.push({heur(start),0,0});

        const Action acts[6]={{0,false},{-1,false},{1,false},{0,true},{-1,true},{1,true}};
        const int POP_LIMIT = wantFrontier ? ((isJump&&K>=6000)?900:(K>=6000?1800:(K>=3000?2800:3800)))
                                           : ((isJump&&K>=6000)?2200:(K>=6000?3600:(K>=3000?5200:7000)));
        int popped=0, found=-1, riskyFound=-1, riskyScore=INT_MAX;
        int startUnknown=viewUnknownCount(start.y,start.x);
        int startR=clamp((start.y+3)/6,0,N-1);

        while(!pq.empty() && popped<POP_LIMIT){
            auto [f,g,ni]=pq.top(); pq.pop();
            auto &nd=nodes[ni];
            uint64_t ck=searchKey(nd.s,nd.usedHaz);
            auto it=bestG.find(ck);
            if(it==bestG.end() || it->second!=g) continue;
            popped++;

            if(!wantFrontier && nd.hitTarget){
                found=ni;
                break;
            }
            if(wantFrontier && ni!=0){
                int gain=viewUnknownCount(nd.s.y,nd.s.x);
                if(gain>=max(48,startUnknown+24)){
                    int rr=clamp((nd.s.y+3)/6,0,N-1);
                    int cc=clamp((nd.s.x+3)/6,0,M-1);
                    int risk=cellTrapPenalty(rr,cc,startR);
                    int score=nd.g+risk-min(220,gain/2);
                    if(score<riskyScore){ riskyScore=score; riskyFound=ni; }
                    // Prefer viewpoints that do not require a blind one-way drop into
                    // a low-degree pocket. Risky viewpoints remain a last resort.
                    if(risk<=75){
                        found=ni;
                        break;
                    }
                }
            }

            Motion cur=nd.s;
            for(Action a:acts){
                Motion nx;
                bool hit=false;
                int haz=0;
                if(!stepState(cur,a,nx,targetZ,hit,haz)) continue;

                int nused=min(HAZCAP,nd.usedHaz+haz);
                if(nused>maxHazEntries) continue;
                int oldShieldUsed=min(nd.usedHaz,shieldStart);
                int newShieldUsed=min(nused,shieldStart);
                int oldDamage=max(0,nd.usedHaz-shieldStart);
                int newDamage=max(0,nused-shieldStart);

                // Frames are free in the score, so prefer safe detours strongly.
                // Still finite: if a spike is truly unavoidable, the path remains legal.
                int edge=1;
                edge += (newShieldUsed-oldShieldUsed)*20;
                edge += (newDamage-oldDamage)*260;

                // Once a normal jump has started, strongly prefer holding J for both
                // available hold frames. Short jumps remain possible when really useful.
                if(cur.prevJumped && cur.hold>0 && !a.J) edge+=4;

                int ng=g+edge;
                uint64_t key=searchKey(nx,nused);
                auto jt=bestG.find(key);
                if(jt!=bestG.end() && jt->second<=ng) continue;

                bestG[key]=ng;
                int idx=(int)nodes.size();
                nodes.push_back({nx,nused,ni,a,ng,hit});
                pq.push({ng+heur(nx),ng,idx});
            }
        }

        if(found==-1 && wantFrontier && riskyFound!=-1 && riskyScore<320) found=riskyFound;
        if(found==-1) return {};

        vector<Action> rev;
        for(int v=found;nodes[v].parent!=-1;v=nodes[v].parent)
            rev.push_back(nodes[v].act);
        reverse(rev.begin(),rev.end());

        deque<Action> res;
        for(Action a:rev) res.push_back(a);
        return res;
    };

    // Cell-level pushability used only by the optimistic box-route layer.
    auto coarsePushableBox=[&](int r,int c,int dir)->bool{
        if(dir==0 || !insideCell(r,c) || !cellSeen[cid(r,c)]) return false;
        if(world[6*r+3][6*c+3]!='O') return false;
        int cc=c;
        while(insideCell(r,cc) && cellSeen[cid(r,cc)] && world[6*r+3][6*cc+3]=='O') cc+=dir;
        if(!insideCell(r,cc) || !cellSeen[cid(r,cc)]) return false;
        char dst=world[6*r+3][6*cc+3];
        return dst!='#' && dst!='O';
    };

    struct CoarseRouteInfo{
        bool ok=false;
        int cost=0;
        int hazardCells=0;
        int boxCells=0;
        int nextR=-1,nextC=-1;
    };

    // Coarse route used for deliberate box clearing.  Spikes can be forbidden.
    // Boxes are traversable only when approached horizontally and plausibly pushable.
    auto coarseRouteTo=[&](int sr,int sc,int target,int shieldNow,
                           bool forbidSpikes=false)->CoarseRouteInfo{
        const int INF=1e9;
        vector<int> dist(K,INF),par(K,-1),hz(K,INF),bx(K,INF);
        using Q=tuple<int,int,int,int>; // cost,haz,box,node
        priority_queue<Q,vector<Q>,greater<Q>> pq;
        if(!insideCell(sr,sc) || target<0 || target>=K) return {};
        int st=cid(sr,sc);
        dist[st]=0; hz[st]=0; bx[st]=0; par[st]=st;
        pq.push({0,0,0,st});
        static const int dr[4]={0,0,-1,1};
        static const int dc[4]={-1,1,0,0};
        while(!pq.empty()){
            auto [cd,chz,cbx,v]=pq.top(); pq.pop();
            if(cd!=dist[v] || chz!=hz[v] || cbx!=bx[v]) continue;
            if(v==target) break;
            int r=v/M,c=v%M;
            for(int k=0;k<4;k++){
                int nr=r+dr[k],nc=c+dc[k];
                if(!insideCell(nr,nc) || !cellSeen[cid(nr,nc)]) continue;
                int nv=cid(nr,nc);
                char ch=world[6*nr+3][6*nc+3];
                if(ch=='#') continue;
                if(ch=='O'){
                    // A box is not a generic passable cell: it is a directed push edge.
                    if(dr[k]!=0 || !coarsePushableBox(nr,nc,dc[k])) continue;
                }
                if(ch=='^' && forbidSpikes) continue;

                int nhz=chz+(ch=='^');
                int nbx=cbx+(ch=='O');
                int w=10;
                // Clearing a pushable box is deliberately cheaper than touching a spike.
                if(ch=='O') w+=(isPits?1:10);
                if(ch=='^'){
                    int shieldLeft=max(0,shieldNow-chz);
                    w+=(shieldLeft>0?140:1400);
                }
                if(nr>r) w+=3;
                if(nr<r) w+=6;
                int nd=cd+w;
                // Lexicographic tie break: fewer hazards first, then fewer boxes.
                if(nd<dist[nv] ||
                   (nd==dist[nv] && pair<int,int>{nhz,nbx}<pair<int,int>{hz[nv],bx[nv]})){
                    dist[nv]=nd; hz[nv]=nhz; bx[nv]=nbx; par[nv]=v;
                    pq.push({nd,nhz,nbx,nv});
                }
            }
        }
        CoarseRouteInfo res;
        if(dist[target]>=INF/2) return res;
        res.ok=true; res.cost=dist[target]; res.hazardCells=hz[target]; res.boxCells=bx[target];
        int v=target;
        while(par[v]!=st && par[v]!=v && par[v]!=-1) v=par[v];
        if(par[v]==-1) return CoarseRouteInfo{};
        res.nextR=v/M; res.nextC=v%M;
        return res;
    };

    // Safest coarse route to any known frontier.  Used before we ever allow a bare
    // spike crossing, so exploration cannot casually spend health while a box/safe
    // route to new information exists.
    auto coarseRouteToFrontier=[&](int sr,int sc,int shieldNow,bool forbidSpikes)->CoarseRouteInfo{
        const int INF=1e9;
        CoarseRouteInfo res;
        if(!insideCell(sr,sc)) return res;
        vector<int> dist(K,INF),par(K,-1),hz(K,INF),bx(K,INF);
        using Q=tuple<int,int,int,int>;
        priority_queue<Q,vector<Q>,greater<Q>> pq;
        int st=cid(sr,sc);
        dist[st]=0; hz[st]=0; bx[st]=0; par[st]=st;
        pq.push({0,0,0,st});
        static const int dr[4]={0,0,-1,1};
        static const int dc[4]={-1,1,0,0};
        int best=-1,bestScore=INF;
        while(!pq.empty()){
            auto [cd,chz,cbx,v]=pq.top(); pq.pop();
            if(cd!=dist[v] || chz!=hz[v] || cbx!=bx[v]) continue;
            int r=v/M,c=v%M;
            bool frontier=false;
            for(int k=0;k<4;k++){
                int nr=r+dr[k],nc=c+dc[k];
                if(insideCell(nr,nc) && !cellSeen[cid(nr,nc)]) frontier=true;
            }
            if(v!=st && frontier){
                int score=cd+cellTrapPenalty(r,c,sr)+min(80,getVisit(v));
                if(score<bestScore){ bestScore=score; best=v; }
            }
            for(int k=0;k<4;k++){
                int nr=r+dr[k],nc=c+dc[k];
                if(!insideCell(nr,nc) || !cellSeen[cid(nr,nc)]) continue;
                int nv=cid(nr,nc);
                char ch=world[6*nr+3][6*nc+3];
                if(ch=='#') continue;
                if(ch=='O' && (dr[k]!=0 || !coarsePushableBox(nr,nc,dc[k]))) continue;
                if(ch=='^' && forbidSpikes) continue;
                int nhz=chz+(ch=='^');
                int nbx=cbx+(ch=='O');
                int w=10+min(30,getVisit(nv));
                if(ch=='O') w+=(isPits?1:10);
                if(ch=='^'){
                    int shieldLeft=max(0,shieldNow-chz);
                    w+=(shieldLeft>0?140:1400);
                }
                int nd=cd+w;
                if(nd<dist[nv] ||
                   (nd==dist[nv] && pair<int,int>{nhz,nbx}<pair<int,int>{hz[nv],bx[nv]})){
                    dist[nv]=nd; hz[nv]=nhz; bx[nv]=nbx; par[nv]=v;
                    pq.push({nd,nhz,nbx,nv});
                }
            }
        }
        if(best==-1) return res;
        res.ok=true; res.cost=dist[best]; res.hazardCells=hz[best]; res.boxCells=bx[best];
        int v=best;
        while(par[v]!=st && par[v]!=v && par[v]!=-1) v=par[v];
        if(par[v]==-1) return CoarseRouteInfo{};
        res.nextR=v/M; res.nextC=v%M;
        return res;
    };

    int prevCoins=0;
    int prevShield=-1;

    for(int t=0;;t++){
        int coins,shield,y,x; string enc;
        if(!(cin>>coins>>shield>>y>>x>>enc)) return 0;
        auto scr=decode(enc);
        if(t==10000){ cout<<"FINISH\n"<<flush; return 0; }
        bool shieldStateChanged=(prevShield!=-1 && shield!=prevShield);

        // The previous command is now observable.  If J was issued while we are still
        // grounded and y did not rise, repeating J at the same place is useless (typically
        // a ceiling / one-way shaft mistaken for an upward route).
        bool observedGroundedTop=false;
        for(int sx=21;sx<=26;sx++){
            char ch=scr[27][sx];
            if(ch=='#' || ch=='O'){ observedGroundedTop=true; break; }
        }
        int observedDy=(prevActualY==-1?0:y-prevActualY);
        if(haveLastIssued && lastIssued.J && observedGroundedTop && observedDy>=0){
            failedJumpStreak++;
        }else if(!haveLastIssued || !lastIssued.J || observedDy<0){
            failedJumpStreak=0;
        }
        if(failedJumpStreak>=2){
            // Pick the side with more immediate horizontal clearance, with fallbackDir as tie-break.
            auto clearance=[&](int dir){
                int best=0;
                for(int d=1;d<=20;d++){
                    int col=(dir<0?21-d:26+d);
                    if(col<0||col>=48) break;
                    bool block=false;
                    for(int rr=21;rr<=26;rr++) if(scr[rr][col]=='#'||scr[rr][col]=='O'){ block=true; break; }
                    if(block) break;
                    best=d;
                }
                return best;
            };
            int lc=clearance(-1), rc=clearance(1);
            jumpEscapeDir=(rc>lc?1:(lc>rc?-1:fallbackDir));
            jumpBanUntil=t+24;
            failedJumpStreak=0;
            plan.clear();
            plannedTarget=-1;
            plannedKind=0;
            coarseGuidance=false;
        }

        // Reconcile our internal physical state with the observed position.
        if(!haveMotion){
            motion={y,x,0,0,false,false,0};
            haveMotion=true;
        }else if(havePrediction && predicted.y==y && predicted.x==x){
            motion=predicted;
            mismatchStreak=0;
        }else{
            // Dynamic boxes or an imperfect remembered map can invalidate a static prediction.
            // Preserve position, but infer momentum conservatively from the observed displacement.
            mismatchStreak++;
            int dx=(prevActualX==-1?0:x-prevActualX);
            int dy=(prevActualY==-1?0:y-prevActualY);
            motion.y=y; motion.x=x;
            motion.vx=(abs(dx)<=4?dx:0);
            if(dy<0) motion.vy=max(-3,dy+1);
            else if(dy>0) motion.vy=min(6,dy+1);
            else motion.vy=0;
            motion.prevJumpTime=false;
            motion.prevJumped=false;
            motion.hold=0;
            plan.clear();
            plannedTarget=-1;
            plannedKind=0;
            coarseGuidance=false;
        }
        havePrediction=false;

        int oy=y-21, ox=x-21;
        bool discoveredNewCoin=false;
        bool discoveredNewShield=false;
        bool discoveredNewCell=false;

        // Pixel-accurate global map update. '@' hides lower layers, so preserve remembered data.
        for(int sy=0;sy<48;sy++) for(int sx=0;sx<48;sx++){
            int gy=oy+sy,gx=ox+sx;
            if(!insidePix(gy,gx)) continue;
            char ch=scr[sy][sx];
            if(ch=='@'){
                if(world[gy][gx]=='?') world[gy][gx]='.';
            }else{
                world[gy][gx]=ch;
            }
            if(ch=='$'){
                int z=cid(gy/6,gx/6);
                if(coinAlive[z]!=1) discoveredNewCoin=true;
                coinAlive[z]=1;
            }
            if(ch=='*'){
                int z=cid(gy/6,gx/6);
                if(shieldAlive[z]!=1) discoveredNewShield=true;
                shieldAlive[z]=1;
            }
        }

        // Mark every completely visible logical cell as explored, and clear disappeared coins
        // only when no box is hiding the cell.
        int cr=(y+3)/6,cc=(x+3)/6;
        for(int r=max(0,cr-6);r<=min(N-1,cr+6);r++) for(int c=max(0,cc-6);c<=min(M-1,cc+6);c++){
            int gy=6*r,gx=6*c;
            if(gy<oy||gy+5>oy+47||gx<ox||gx+5>ox+47) continue;
            if(!cellSeen[cid(r,c)]){ cellSeen[cid(r,c)]=1; seenCells++; discoveredNewCell=true; }
            bool at=false,box=false,coin=false,shieldItem=false;
            for(int py=0;py<6;py++) for(int px=0;px<6;px++){
                char ch=scr[gy+py-oy][gx+px-ox];
                at|=(ch=='@');
                box|=(ch=='O');
                coin|=(ch=='$');
                shieldItem|=(ch=='*');
            }

            int z=cid(r,c);
            if(coin) coinAlive[z]=1;
            else if(!at && !box && coinAlive[z]==1) coinAlive[z]=0;
            else if(!at && coinAlive[z]==-1) coinAlive[z]=0;

            if(shieldItem) shieldAlive[z]=1;
            else if(!at && !box && shieldAlive[z]==1) shieldAlive[z]=0;
            else if(!at && shieldAlive[z]==-1) shieldAlive[z]=0;
        }

        // ------------------------------------------------------------------
        // Dedicated Choice branch selector.
        //
        // Choice is not an "individual shield" problem.  At a top ledge we enumerate
        // the ACTUAL one-cell-wide holes that the 6px player can fall through, group the
        // visible shield/spike events below each hole by column, and simulate them in
        // vertical order using the judge semantics:
        //   spike: shield>0 ? --shield : ++bareDamage   (never negative)
        //   shield item: ++shield
        // Once a chute is selected it is locked until the fall ends, so collecting a
        // shield halfway down cannot make the generic planner switch to another branch.
        bool choiceSpecialNow=false;
        int choiceBestCol=-1;

        if(isChoice){
            int curR=clamp((y+3)/6,0,N-1);
            int curC=clamp((x+3)/6,0,M-1);

            auto choiceHardPix=[&](int py,int px){
                if(!insidePix(py,px)) return true;
                char ch=world[py][px];
                return ch=='#' || ch=='O' || ch=='?';
            };
            auto choiceCanOccupyX=[&](int tx){
                if(tx<0 || tx+5>=W) return false;
                for(int py=y;py<=y+5;py++)
                    for(int px=tx;px<=tx+5;px++)
                        if(choiceHardPix(py,px)) return false;
                return true;
            };
            auto choiceHorizReach=[&](int tx){
                if(!choiceCanOccupyX(tx)) return false;
                int cx=x, d=(tx>cx)-(tx<cx);
                while(cx!=tx){
                    int nx=cx+d;
                    int edge=(d>0?nx+5:nx);
                    for(int py=y;py<=y+5;py++) if(choiceHardPix(py,edge)) return false;
                    cx=nx;
                }
                return true;
            };
            auto choiceCanDrop=[&](int tx){
                if(!choiceCanOccupyX(tx) || y+6>=H) return false;
                for(int px=tx;px<=tx+5;px++) if(choiceHardPix(y+6,px)) return false;
                return true;
            };

            // If a locked fall has clearly landed, release it before looking for the next fork.
            if(choiceDropCol!=-1 && choiceDropStarted && observedGroundedTop && observedDy==0 &&
               choiceDropStartY!=-1 && y>=choiceDropStartY+5){
                // If a mandatory-coin chute somehow landed without increasing the coin
                // counter, force an immediate fresh generic replan instead of treating
                // the old fork decision as successfully completed.
                bool missedMandatoryCoin=choiceDropHasCoin && !choiceDropCoinCollected;
                choiceDropCol=-1;
                choiceDropStartY=-1;
                choiceDropSince=-1;
                choiceDropStarted=false;
                choiceDropHasCoin=false;
                choiceDropCoinCollected=false;
                plan.clear();
                plannedTarget=-1;
                plannedKind=0;
                coarseGuidance=false;
                retryAt=t;
                if(missedMandatoryCoin) failedSeenCells=min(failedSeenCells,seenCells-6);
            }
            if(choiceDropCol!=-1 && choiceDropSince!=-1 &&
               t-choiceDropSince>(choiceDropHasCoin?260:140)){
                // Safety valve for a malformed/partially-known fork; do not lock forever.
                choiceDropCol=-1;
                choiceDropStartY=-1;
                choiceDropSince=-1;
                choiceDropStarted=false;
                choiceDropHasCoin=false;
                choiceDropCoinCollected=false;
            }

            if(choiceDropCol!=-1){
                choiceSpecialNow=true;
                choiceBestCol=choiceDropCol;
                if((observedDy>0 && y>choiceDropStartY) || motion.vy>0) choiceDropStarted=true;
            }else if(motion.vy<=1){
                // A Choice fork is often reached while still rising from a + shaft, so
                // requiring grounded=true misses the actual decision frame.  Geometry
                // (horizontal reachability + a real drop immediately below) is enough.
                struct CBranch{
                    int col=-1;
                    int damage=INT_MAX;
                    int finalShield=-1;
                    int pickups=-1;
                    int coins=0;
                    int spikes=0;
                    int firstEventRow=INT_MAX;
                    int lastEventRow=-1;
                    int distX=INT_MAX;
                };
                vector<CBranch> branches;

                // Only fully visible entry x positions are candidates.  The player is 6px
                // wide, so a one-cell chute requires top-left x == 6*c exactly.
                int cLo=max(0,(ox-6)/6);
                int cHi=min(M-1,(ox+53)/6);
                int visibleBottom=min(N-1,(oy+47)/6);
                int scanStart=max(0,(y+5)/6);

                for(int c=cLo;c<=cHi;c++){
                    int tx=6*c;
                    if(tx<ox || tx+5>ox+47) continue;
                    if(abs(c-curC)>7) continue;
                    if(!choiceHorizReach(tx) || !choiceCanDrop(tx)) continue;

                    int simShield=shield;
                    int damage=0,pickups=0,spikes=0;
                    int firstEvent=INT_MAX,lastEvent=-1;
                    bool started=false,blockedBeforeEvent=false;
                    bool shaftWallEvidence=false;

                    for(int r=scanStart;r<=visibleBottom;r++){
                        if(!insideCell(r,c) || !cellSeen[cid(r,c)]) break;
                        char ch=world[6*r+3][6*c+3];
                        if(ch=='#' || ch=='O'){
                            if(!started) blockedBeforeEvent=true;
                            break;
                        }

                        bool lw=(c==0) || (cellSeen[cid(r,c-1)] &&
                            (world[6*r+3][6*(c-1)+3]=='#' || world[6*r+3][6*(c-1)+3]=='O'));
                        bool rw=(c+1==M) || (cellSeen[cid(r,c+1)] &&
                            (world[6*r+3][6*(c+1)+3]=='#' || world[6*r+3][6*(c+1)+3]=='O'));
                        if(lw || rw) shaftWallEvidence=true;

                        int z=cid(r,c);
                        if(ch=='^'){
                            started=true;
                            firstEvent=min(firstEvent,r); lastEvent=r; spikes++;
                            if(simShield>0) --simShield;
                            else ++damage;
                        }
                        if(shieldAlive[z]==1){
                            started=true;
                            firstEvent=min(firstEvent,r); lastEvent=r; pickups++;
                            ++simShield;
                        }
                        if(coinAlive[z]==1){
                            started=true;
                            firstEvent=min(firstEvent,r); lastEvent=r; coins++;
                        }
                    }
                    // A Choice chute may contain a coin and no shield at all.  Such a
                    // chute is still a mandatory/highest-priority branch.
                    if(blockedBeforeEvent || (pickups==0 && coins==0) || firstEvent==INT_MAX) continue;
                    if(!shaftWallEvidence) continue;
                    // A Choice fork must start below/near the current ledge; ignore old
                    // resource columns far above/below that merely remain in world memory.
                    if(firstEvent>curR+7 || lastEvent<curR-1) continue;

                    CBranch b;
                    b.col=c; b.damage=damage; b.finalShield=simShield;
                    b.pickups=pickups; b.coins=coins; b.spikes=spikes; b.firstEventRow=firstEvent;
                    b.lastEventRow=lastEvent; b.distX=abs(tx-x);
                    branches.push_back(b);
                }

                // Mandatory-coin override.  Do NOT require the full "shaft wall"
                // signature for a coin chute: in the Choice generator the mandatory coin
                // can be visible before the second branch/walls are fully revealed.
                // If a known coin lies below a horizontally reachable drop mouth and the
                // column to it is continuously open, that chute wins immediately.
                vector<CBranch> coinBranches;
                for(int c=cLo;c<=cHi;c++){
                    int tx=6*c;
                    if(tx<ox || tx+5>ox+47) continue;
                    if(abs(c-curC)>7) continue;
                    if(!choiceHorizReach(tx) || !choiceCanDrop(tx)) continue;

                    int coinRow=-1;
                    for(int r=scanStart;r<=visibleBottom;r++){
                        if(!insideCell(r,c) || !cellSeen[cid(r,c)]) break;
                        char ch=world[6*r+3][6*c+3];
                        if(ch=='#' || ch=='O') break;
                        if(coinAlive[cid(r,c)]==1){ coinRow=r; break; }
                    }
                    if(coinRow==-1 || coinRow>curR+7) continue;

                    int simShield=shield,damage=0,pickups=0,spikes=0,coinsInCol=0;
                    int firstEvent=INT_MAX,lastEvent=-1;
                    for(int r=scanStart;r<=visibleBottom;r++){
                        if(!insideCell(r,c) || !cellSeen[cid(r,c)]) break;
                        char ch=world[6*r+3][6*c+3];
                        if(ch=='#' || ch=='O') break;
                        int z=cid(r,c);
                        if(ch=='^'){
                            firstEvent=min(firstEvent,r); lastEvent=r; ++spikes;
                            if(simShield>0) --simShield; else ++damage;
                        }
                        if(shieldAlive[z]==1){
                            firstEvent=min(firstEvent,r); lastEvent=r; ++pickups; ++simShield;
                        }
                        if(coinAlive[z]==1){
                            firstEvent=min(firstEvent,r); lastEvent=r; ++coinsInCol;
                        }
                    }
                    CBranch b;
                    b.col=c; b.damage=damage; b.finalShield=simShield;
                    b.pickups=pickups; b.coins=coinsInCol; b.spikes=spikes;
                    b.firstEventRow=firstEvent; b.lastEventRow=lastEvent; b.distX=abs(tx-x);
                    coinBranches.push_back(b);
                }

                auto betterChoice=[&](const CBranch&a,const CBranch&b){
                    if(a.coins!=b.coins) return a.coins>b.coins;
                    if(a.damage!=b.damage) return a.damage<b.damage;
                    if(a.finalShield!=b.finalShield) return a.finalShield>b.finalShield;
                    if(a.pickups!=b.pickups) return a.pickups>b.pickups;
                    if(a.spikes!=b.spikes) return a.spikes<b.spikes;
                    if(a.distX!=b.distX) return a.distX<b.distX;
                    return a.col<b.col;
                };

                bool chooseNow=false;
                CBranch best;
                if(!coinBranches.empty()){
                    best=coinBranches[0];
                    for(size_t i=1;i<coinBranches.size();i++)
                        if(betterChoice(coinBranches[i],best)) best=coinBranches[i];
                    chooseNow=true;
                }else if(branches.size()>=2){
                    best=branches[0];
                    for(size_t i=1;i<branches.size();i++)
                        if(betterChoice(branches[i],best)) best=branches[i];
                    chooseNow=true;
                }

                if(chooseNow){
                    choiceDropCol=best.col;
                    choiceDropStartY=y;
                    choiceDropSince=t;
                    choiceDropStarted=false;
                    choiceDropHasCoin=(best.coins>0);
                    choiceDropCoinCollected=false;
                    choiceBestCol=best.col;
                    choiceSpecialNow=true;

                    // A fork decision is atomic.  Generic plans may not steal control
                    // between choosing the chute and actually landing below it.
                    plan.clear();
                    plannedTarget=-1;
                    plannedKind=0;
                    coarseGuidance=false;
                    boxCommitTarget=-1;
                    boxCommitKind=0;
                    choiceColumnLock=-1;
                    escapeUntil=-1;
                }
            }
        }

        bool gotCoin=coins>prevCoins;
        bool targetGone=false;
        if(plannedTarget!=-1){
            if(plannedKind=='$' && coinAlive[plannedTarget]==0) targetGone=true;
            if(plannedKind=='*' && shieldAlive[plannedTarget]==0) targetGone=true;
        }

        if(gotCoin){
            if(isChoice && choiceDropCol!=-1) choiceDropCoinCollected=true;
            if(plannedKind=='$' && plannedTarget!=-1) coinAlive[plannedTarget]=0;
            // Also clear any remembered coin in the immediate swept vicinity.
            for(int r=max(0,(y-6)/6);r<=min(N-1,(y+11)/6);r++) for(int c=max(0,(x-6)/6);c<=min(M-1,(x+11)/6);c++){
                int z=cid(r,c);
                if(coinAlive[z]!=1) continue;
                int cy=6*r+3,cx=6*c+3;
                if(abs(cy-(y+3))<=9 && abs(cx-(x+3))<=9) coinAlive[z]=0;
            }
        }

        // A collected coin starts a new navigation phase.
        // Forget only the "I have already walked here a lot" bias in O(1) via an epoch;
        // the learned geometry itself is preserved.
        if(gotCoin){
            ++currentVisitEpoch;
            if(currentVisitEpoch==INT_MAX){
                fill(visitEpoch.begin(),visitEpoch.end(),0);
                currentVisitEpoch=1;
            }
            fallbackGoal=-1;
            fallbackGoalSince=t;
            fallbackGoalSeenCells=seenCells;
            fallbackLock=0;
            xStall=0;
            escapeUntil=-1;
            lastDiscoveryT=t;
        }

        if(gotCoin || targetGone){
            if(plannedTarget==boxCommitTarget){ boxCommitTarget=-1; boxCommitKind=0; }
            plan.clear();
            plannedTarget=-1;
            plannedKind=0;
            coarseGuidance=false;
        }

        // Choice decisions depend on the CURRENT shield count and on newly revealed
        // shield branches.  Do not finish an old generic plan after either changes.
        if(isChoice && (discoveredNewShield || shieldStateChanged)){
            plan.clear();
            plannedTarget=-1;
            plannedKind=0;
            coarseGuidance=false;
            boxCommitTarget=-1;
            boxCommitKind=0;
            retryAt=t;
        }

        prevCoins=coins;
        prevShield=shield;
        if(gotCoin || discoveredNewCoin || discoveredNewShield || targetGone) retryAt=t;
        if(discoveredNewCell || discoveredNewCoin || discoveredNewShield || gotCoin)
            lastDiscoveryT=t;

        if(prevActualY==y && prevActualX==x) noProgress++; else noProgress=0;
        prevActualY=y; prevActualX=x;
        if(prevFallbackX==x) xStall++; else xStall=0;
        prevFallbackX=x;

        int visitR=clamp((y+3)/6,0,N-1), visitC=clamp((x+3)/6,0,M-1);
        addVisit(cid(visitR,visitC));

        if(discoveredNewCell){
            fallbackGoalSince=t;
            fallbackGoalSeenCells=seenCells;
        }
        if(fallbackGoal!=-1 && t-fallbackGoalSince>55 &&
           seenCells==fallbackGoalSeenCells){
            badFrontierUntil[fallbackGoal]=t+180;
            fallbackGoal=-1;
            fallbackGoalSince=t;
            fallbackGoalSeenCells=seenCells;
        }

        // If a plan got stuck despite exact planning, temporarily abandon that target.
        if(noProgress>20){
            if(plannedTarget!=-1) badTargetUntil[plannedTarget]=t+180;
            plan.clear();
            plannedTarget=-1;
            plannedKind=0;
            coarseGuidance=false;
            boxCommitTarget=-1; boxCommitKind=0;
        }

        // If we have learned nothing for a long time, a formally valid local plan may
        // still be cycling around the same wall/platform. Break that commitment.
        if(!lightFSM && t-lastDiscoveryT>80 && t>=escapeUntil){
            int leftVisits=0,rightVisits=0;
            int rr=clamp((y+3)/6,0,N-1), cc=clamp((x+3)/6,0,M-1);
            for(int d=1;d<=8;d++){
                if(cc-d>=0) leftVisits+=getVisit(cid(rr,cc-d));
                if(cc+d<M) rightVisits+=getVisit(cid(rr,cc+d));
            }
            escapeDir=(leftVisits<=rightVisits?-1:1);
            escapeUntil=t+36;
            plan.clear();
            plannedTarget=-1;
            plannedKind=0;
            coarseGuidance=false;
            boxCommitTarget=-1; boxCommitKind=0;
            lastDiscoveryT=t;
        }

        // Replan in strict safety tiers.  A spike path is NEVER accepted while a
        // spike-free exact/box/shield/frontier route is known.  This ordering is more
        // important than small utility differences.
        if(!lightFSM && !choiceSpecialNow && plan.empty() && !coarseGuidance && t>=escapeUntil && (double)clock()/CLOCKS_PER_SEC < PLAN_CPU_BUDGET){
            bool retryByKnowledge=(seenCells>=failedSeenCells+6);
            bool shouldRetry=(t>=retryAt || retryByKnowledge ||
                              discoveredNewCoin || discoveredNewShield || gotCoin || targetGone ||
                              boxCommitTarget!=-1);

            if(shouldRetry){
                buildPrefix();
                int sr=clamp((y+3)/6,0,N-1);
                int sc=clamp((x+3)/6,0,M-1);

                struct Cand{
                    int z=-1;
                    char kind=0;
                    int utility=0;
                    int chain=0;
                };
                vector<Cand> coinsCand, shieldsCand;
                vector<int> knownShields;
                for(int z=0;z<K;z++) if(shieldAlive[z]==1) knownShields.push_back(z);

                // One spike-free coarse shortest-path tree from the current position.
                // Every known shield is attributed to the FIRST branch taken from here.
                // Therefore a fork containing two shields is valued above a fork containing one.
                const int INF2=1e9;
                vector<int> safeDist(K,INF2), firstStep(K,-1);
                using BQ=pair<int,int>;
                priority_queue<BQ,vector<BQ>,greater<BQ>> bpq;
                int bst=cid(sr,sc);
                safeDist[bst]=0; firstStep[bst]=bst;
                // Jump has no shield-branch decision.  This Dijkstra does not affect the
                // coin ordering there, so skip it entirely on Jump maps.
                if(!isJump) bpq.push({0,bst});
                static const int bdr[4]={0,0,-1,1};
                static const int bdc[4]={-1,1,0,0};
                while(!bpq.empty()){
                    auto [cd,v]=bpq.top(); bpq.pop();
                    if(cd!=safeDist[v]) continue;
                    int r=v/M,c=v%M;
                    for(int k=0;k<4;k++){
                        int nr=r+bdr[k],nc=c+bdc[k];
                        if(!insideCell(nr,nc) || !cellSeen[cid(nr,nc)]) continue;
                        int nv=cid(nr,nc);
                        char ch=world[6*nr+3][6*nc+3];
                        if(ch=='#' || ch=='^') continue;
                        int w=10;
                        if(ch=='O'){
                            if(bdr[k]!=0 || !coarsePushableBox(nr,nc,bdc[k])) continue;
                            w+=10;
                        }
                        int nd=cd+w;
                        int nf=(v==bst?nv:firstStep[v]);
                        if(nd<safeDist[nv]){
                            safeDist[nv]=nd;
                            firstStep[nv]=nf;
                            bpq.push({nd,nv});
                        }
                    }
                }
                vector<int> branchShieldValue(K,0);
                for(int z:knownShields){
                    if(firstStep[z]==-1 || safeDist[z]>=INF2/2) continue;
                    branchShieldValue[firstStep[z]] += max(120,520-safeDist[z]);
                }

                // Choice-specific branch evaluation.
                // A Choice branch is a vertical drop shaft.  The old version ran a 4-neighbour
                // BFS and therefore invented impossible "upward walking" routes through empty
                // air.  Evaluate the actual vertical order of objects in each known shaft.
                // For each column containing shields, scan the complete known open vertical run,
                // simulate spike/shield events from top to bottom, and choose lexicographically:
                //   1) fewer bare spike hits, 2) more shields left after the drop,
                //   3) more shield pickups.
                // Shield count is clamped by simulation semantics: a spike at zero adds damage
                // but NEVER makes the shield count negative.
                int choicePreferredShield=-1;
                int choiceHazToFirst=0;
                int choicePreferredCol=-1;
                if(false && isChoice && !knownShields.empty()){
                    struct CS {
                        int damage=INT_MAX;
                        int finalShield=-1;
                        int pickups=-1;
                        int firstRow=INT_MAX;
                        int col=-1;
                        int firstShield=-1;
                        int hazToFirst=0;
                    } bestCS;

                    vector<unsigned char> triedCol(M,0);
                    for(int seedShield:knownShields){
                        int c=seedShield%M;
                        if(triedCol[c]) continue;
                        triedCol[c]=1;

                        // Find all alive shields in this column first.
                        vector<int> shieldRows;
                        for(int r=0;r<N;r++){
                            int z=cid(r,c);
                            if(cellSeen[z] && shieldAlive[z]==1) shieldRows.push_back(r);
                        }
                        if(shieldRows.empty()) continue;

                        int anchor=shieldRows.front();
                        int top=anchor, bot=anchor;
                        bool topClosed=false, botClosed=false;
                        while(top>0){
                            int nz=cid(top-1,c);
                            if(!cellSeen[nz]) break;
                            char ch=world[6*(top-1)+3][6*c+3];
                            if(ch=='#' || ch=='O'){ topClosed=true; break; }
                            --top;
                        }
                        if(top==0) topClosed=true;
                        while(bot+1<N){
                            int nz=cid(bot+1,c);
                            if(!cellSeen[nz]) break;
                            char ch=world[6*(bot+1)+3][6*c+3];
                            if(ch=='#' || ch=='O'){ botClosed=true; break; }
                            ++bot;
                        }
                        if(bot==N-1) botClosed=true;

                        // Do not decide from a half-seen shaft; that is exactly how the old
                        // heuristic picked a one-shield branch before seeing the better one.
                        if(!topClosed || !botClosed) continue;

                        int simShield=shield;
                        int damage=0, pickups=0, hazards=0;
                        int firstShield=-1, firstRow=INT_MAX, hazardsToFirst=0;
                        bool shaftLike=false;
                        for(int r=top;r<=bot;r++){
                            int z=cid(r,c);
                            char ch=world[6*r+3][6*c+3];

                            // A real Choice chute is vertically constrained for at least part
                            // of its run.  This rejects a shield lying on an ordinary hallway.
                            bool leftWall=(c==0) || (cellSeen[cid(r,c-1)] &&
                                (world[6*r+3][6*(c-1)+3]=='#' || world[6*r+3][6*(c-1)+3]=='O'));
                            bool rightWall=(c+1==M) || (cellSeen[cid(r,c+1)] &&
                                (world[6*r+3][6*(c+1)+3]=='#' || world[6*r+3][6*(c+1)+3]=='O'));
                            if(leftWall || rightWall) shaftLike=true;

                            if(ch=='^'){
                                ++hazards;
                                if(simShield>0) --simShield;
                                else ++damage;
                            }
                            if(shieldAlive[z]==1){
                                ++simShield;
                                ++pickups;
                                if(firstShield==-1){
                                    firstShield=z;
                                    firstRow=r;
                                    hazardsToFirst=hazards;
                                }
                            }
                        }
                        if(!shaftLike || firstShield==-1) continue;

                        CS cur;
                        cur.damage=damage;
                        cur.finalShield=simShield;
                        cur.pickups=pickups;
                        cur.firstRow=firstRow;
                        cur.col=c;
                        cur.firstShield=firstShield;
                        cur.hazToFirst=hazardsToFirst;

                        bool better=false;
                        if(cur.damage!=bestCS.damage) better=cur.damage<bestCS.damage;
                        else if(cur.finalShield!=bestCS.finalShield) better=cur.finalShield>bestCS.finalShield;
                        else if(cur.pickups!=bestCS.pickups) better=cur.pickups>bestCS.pickups;
                        else if(cur.firstRow!=bestCS.firstRow) better=cur.firstRow<bestCS.firstRow;
                        else better=cur.col<bestCS.col;
                        if(better) bestCS=cur;
                    }

                    if(choiceColumnLock!=-1){
                        int first=-1, haz=0;
                        int prNow=clamp((y+3)/6,0,N-1);
                        // Continue downward through the chosen shaft; never switch branches
                        // merely because a shield/spike event changed the live shield count.
                        for(int r=max(0,prNow-1);r<N;r++){
                            int z=cid(r,choiceColumnLock);
                            if(!cellSeen[z]) break;
                            char ch=world[6*r+3][6*choiceColumnLock+3];
                            if(ch=='#' || ch=='O') break;
                            if(ch=='^') ++haz;
                            if(shieldAlive[z]==1){ first=z; break; }
                        }
                        if(first!=-1){
                            choicePreferredShield=first;
                            choiceHazToFirst=haz;
                            choicePreferredCol=choiceColumnLock;
                        }else{
                            choiceColumnLock=-1;
                        }
                    }
                    if(choiceColumnLock==-1 && bestCS.firstShield!=-1){
                        choicePreferredShield=bestCS.firstShield;
                        choiceHazToFirst=bestCS.hazToFirst;
                        choicePreferredCol=bestCS.col;
                    }
                }

                for(int z=0;z<K;z++){
                    if(badTargetUntil[z]>t) continue;
                    int md=abs(z/M-sr)+abs(z%M-sc);
                    if(coinAlive[z]==1){
                        int u=1300-8*md-cellTrapPenalty(z/M,z%M,sr)/3;
                        coinsCand.push_back({z,'$',u,0});
                    }
                    if(shieldAlive[z]==1){
                        int chain=0;
                        if(firstStep[z]!=-1) chain=branchShieldValue[firstStep[z]];
                        // If the shield is not on a currently known safe branch, do not
                        // fake extra value using straight-line distance through walls.
                        int u=650+chain-6*md-cellTrapPenalty(z/M,z%M,sr)/4;
                        // With no shield, known spikes make a safe shield route strategically urgent.
                        if(shield==0) u+=350;
                        if(isChoice) u+=700 + chain/2;
                        shieldsCand.push_back({z,'*',u,chain});
                    }
                }
                auto byUtility=[](const Cand&a,const Cand&b){
                    if(a.utility!=b.utility) return a.utility>b.utility;
                    if(a.chain!=b.chain) return a.chain>b.chain;
                    return a.z<b.z;
                };
                sort(coinsCand.begin(),coinsCand.end(),byUtility);
                sort(shieldsCand.begin(),shieldsCand.end(),byUtility);

                auto chooseExact=[&](const vector<Cand>& vec,int maxHaz)->bool{
                    int tries=0;
                    for(const auto &ca:vec){
                        if((double)clock()/CLOCKS_PER_SEC >= PLAN_CPU_BUDGET) break;
                        if(tries++>=((isJump && K>=6000)?2:6)) break;
                        auto pth=searchPlan(motion,ca.z,shield,false,maxHaz);
                        if(pth.empty()) continue;
                        plan=move(pth);
                        plannedTarget=ca.z;
                        plannedKind=ca.kind;
                        boxCommitTarget=-1;
                        boxCommitKind=0;
                        return true;
                    }
                    return false;
                };

                auto chooseSafeBox=[&](const vector<Cand>& vec)->bool{
                    int tries=0;
                    for(const auto &ca:vec){
                        if(tries++>=10) break;
                        auto rt=coarseRouteTo(sr,sc,ca.z,shield,true);
                        if(!rt.ok || rt.hazardCells!=0 || rt.boxCells<=0) continue;
                        coarseGuidance=true;
                        coarseNextR=rt.nextR;
                        coarseNextC=rt.nextC;
                        plannedTarget=ca.z;
                        plannedKind=ca.kind;
                        boxCommitTarget=ca.z;
                        boxCommitKind=ca.kind;
                        boxCommitUntil=t+120;
                        return true;
                    }
                    return false;
                };

                // If we committed to clearing boxes for a target, keep doing that before
                // reconsidering any spike route.  Recompute one safe step each frame.
                if(boxCommitTarget!=-1 && t<=boxCommitUntil){
                    bool alive=(boxCommitKind=='$'?coinAlive[boxCommitTarget]==1:
                                boxCommitKind=='*'?shieldAlive[boxCommitTarget]==1:false);
                    if(alive){
                        auto rt=coarseRouteTo(sr,sc,boxCommitTarget,shield,true);
                        if(rt.ok && rt.hazardCells==0){
                            if(rt.boxCells>0){
                                coarseGuidance=true;
                                coarseNextR=rt.nextR;
                                coarseNextC=rt.nextC;
                                plannedTarget=boxCommitTarget;
                                plannedKind=boxCommitKind;
                            }else{
                                // Box route has been opened; exact spike-free physics takes over.
                                auto pth=searchPlan(motion,boxCommitTarget,shield,false,0);
                                if(!pth.empty()){
                                    plan=move(pth);
                                    plannedTarget=boxCommitTarget;
                                    plannedKind=boxCommitKind;
                                    boxCommitTarget=-1;
                                    boxCommitKind=0;
                                }
                            }
                        }else{
                            boxCommitTarget=-1;
                            boxCommitKind=0;
                        }
                    }else{
                        boxCommitTarget=-1;
                        boxCommitKind=0;
                    }
                }

                // Choice: commit first to the first shield on the best resource branch.
                // After that shield is actually collected, the next frame uses the judge's
                // real shield count and evaluates the remaining branch again.
                if(false && isChoice && plan.empty() && !coarseGuidance && choicePreferredShield!=-1){
                    auto pth=searchPlan(motion,choicePreferredShield,shield,false,choiceHazToFirst);
                    if(!pth.empty()){
                        plan=move(pth);
                        plannedTarget=choicePreferredShield;
                        plannedKind='*';
                        if(choicePreferredCol!=-1) choiceColumnLock=choicePreferredCol;
                    }else{
                        auto rt=coarseRouteTo(sr,sc,choicePreferredShield,shield,false);
                        if(rt.ok){
                            coarseGuidance=true;
                            coarseNextR=rt.nextR;
                            coarseNextC=rt.nextC;
                            plannedTarget=choicePreferredShield;
                            plannedKind='*';
                            if(choicePreferredCol!=-1) choiceColumnLock=choicePreferredCol;
                        }
                    }
                }

                // Tier 0: totally spike-free concrete rewards.  Boxes are part
                // of this tier, so a pushable-box route always beats a spike route.
                if(plan.empty() && !coarseGuidance){
                    if(shield==0 && !isChoice){
                        if(!chooseExact(shieldsCand,0)) chooseSafeBox(shieldsCand);
                    }
                }
                if(plan.empty() && !coarseGuidance){
                    if(isPits){
                        // Pits is box-first: opening the drop route is usually the actual task.
                        if(!chooseSafeBox(coinsCand)) chooseExact(coinsCand,0);
                    }else{
                        if(!chooseExact(coinsCand,0)) chooseSafeBox(coinsCand);
                    }
                }
                if(plan.empty() && !coarseGuidance && !isChoice){
                    // Choice shields belong to a whole chute, not to an individual-item target.
                    // Other map types keep the old safe-shield greediness.
                    if(!chooseExact(shieldsCand,0)) chooseSafeBox(shieldsCand);
                }

                // Tier 1: if we already own shields, use them for concrete rewards.
                // Shielded spike contact has no h_t penalty, so this comes before blind
                // frontier exploration.  Safe box routes were already exhausted above.
                if(plan.empty() && !coarseGuidance && shield>0){
                    if(isChoice) chooseExact(coinsCand,shield);
                    else if(!chooseExact(coinsCand,shield)) chooseExact(shieldsCand,shield);
                }

                // Tier 2: spike-free exploration, including deliberate box clearing.
                if(plan.empty() && !coarseGuidance){
                    auto f=searchPlan(motion,-1,shield,true,0);
                    if(!f.empty()){
                        plan=move(f);
                        plannedTarget=-1;
                        plannedKind=0;
                    }else{
                        auto rt=coarseRouteToFrontier(sr,sc,shield,true);
                        if(rt.ok && rt.hazardCells==0){
                            coarseGuidance=true;
                            coarseNextR=rt.nextR;
                            coarseNextC=rt.nextC;
                            plannedTarget=-1;
                            plannedKind=0;
                        }
                    }
                }

                // If a shield remains and no safe frontier exists, it may also fund
                // exploration through spikes before we ever accept bare damage.
                if(plan.empty() && !coarseGuidance && shield>0){
                    auto f=searchPlan(motion,-1,shield,true,shield);
                    if(!f.empty()){
                        plan=move(f);
                        plannedTarget=-1;
                        plannedKind=0;
                    }
                }

                // Tier 3: no safe/boxed/shield-funded progress exists.  Only now may we
                // accept an unavoidable bare spike hit, as required by the problem strategy.
                if(plan.empty() && !coarseGuidance){
                    bool picked=false;
                    int tries=0;
                    vector<Cand> all=coinsCand;
                    if(!isChoice) all.insert(all.end(),shieldsCand.begin(),shieldsCand.end());
                    sort(all.begin(),all.end(),byUtility);
                    for(const auto &ca:all){
                        if((double)clock()/CLOCKS_PER_SEC >= PLAN_CPU_BUDGET || tries++>=6) break;
                        auto pth=searchPlan(motion,ca.z,shield,false,32);
                        if(pth.empty()) continue;
                        plan=move(pth);
                        plannedTarget=ca.z;
                        plannedKind=ca.kind;
                        picked=true;
                        break;
                    }
                    if(!picked){
                        auto f=searchPlan(motion,-1,shield,true,32);
                        if(!f.empty()){
                            plan=move(f);
                            plannedTarget=-1;
                            plannedKind=0;
                        }else{
                            retryAt=t+30;
                            failedSeenCells=seenCells;
                        }
                    }
                }
            }
        }

        Action act{0,false};

        // ------------------------------------------------------------
        // Type-specific lightweight policies.
        // ------------------------------------------------------------
        bool specialAct=false;

        auto screenRowSolid=[&](int rr,int l,int r){
            if(rr<0||rr>=48) return true;
            l=max(l,0); r=min(r,47);
            for(int c=l;c<=r;c++) if(scr[rr][c]=='#'||scr[rr][c]=='O') return true;
            return false;
        };
        auto screenColHas=[&](int cc,char ch){
            if(cc<0||cc>=48) return false;
            for(int r=21;r<=26;r++) if(scr[r][cc]==ch) return true;
            return false;
        };
        auto overlapPlusNow=[&](){
            // '@' has higher display priority than '+', so inspect the remembered global map.
            for(int yy=y;yy<=y+5;yy++) for(int xx=x;xx<=x+5;xx++)
                if(insidePix(yy,xx) && world[yy][xx]=='+') return true;
            return false;
        };



        if(isChoice && choiceSpecialNow && choiceBestCol!=-1){
            specialAct=true;
            int tx=6*choiceBestCol;

            // Exact pixel alignment controller for Choice chutes.
            //
            // Important judge detail: with an opposite command, horizontal speed is
            // decelerated FIRST and then accelerated in the opposite direction.  Thus
            // at x==tx,vx==1, pressing '<' does NOT stop us: 1 -> 0 -> -1 and we move
            // one pixel away from the hole.  The old controller therefore oscillated
            // around a one-cell (6 px) chute forever.
            //
            // Instead, test all three commands {-1,0,+1} using the exact horizontal
            // velocity rule and choose the one whose NEXT state is closest to
            // (x=tx,vx=0).  This is O(1) and converges cleanly even from vx=+/-4.
            auto predictChoiceHorizontal=[&](int dir)->pair<int,int>{
                int px=x, pv=motion.vx;

                if(pv!=0 && (dir==0 || (pv>0 ? dir<0 : dir>0)))
                    pv += (pv>0 ? -1 : 1);

                if(dir!=0){
                    int edge=(dir<0 ? px-1 : px+6);
                    bool commandBlocked=(edge<0 || edge>=W);
                    if(!commandBlocked){
                        for(int yy=y;yy<=y+5;yy++){
                            char ch=world[yy][edge];
                            if(ch=='#' || ch=='O' || ch=='?'){
                                commandBlocked=true;
                                break;
                            }
                        }
                    }
                    if(!commandBlocked) pv=max(-4,min(4,pv+dir));
                }

                // Apply horizontal movement pixel-by-pixel, exactly enough for the
                // local steering decision.  A collision zeroes vx.
                int step=(pv>0)-(pv<0);
                for(int k=0;k<abs(pv);k++){
                    int edge=(step<0 ? px-1 : px+6);
                    bool blocked=(edge<0 || edge>=W);
                    if(!blocked){
                        for(int yy=y;yy<=y+5;yy++){
                            char ch=world[yy][edge];
                            if(ch=='#' || ch=='O' || ch=='?'){
                                blocked=true;
                                break;
                            }
                        }
                    }
                    if(blocked){ pv=0; break; }
                    px+=step;
                }
                return {px,pv};
            };

            int bestDir=0;
            long long bestScore=(1LL<<60);
            for(int dir=-1;dir<=1;dir++){
                auto [nx,nv]=predictChoiceHorizontal(dir);
                long long posErr=llabs((long long)nx-tx);
                long long score=100*posErr + 7*abs(nv);

                // Crossing the target with momentum is much worse than stopping one
                // pixel short: it tends to bounce between the two lips of the chute.
                if((x<tx && nx>tx) || (x>tx && nx<tx)) score+=35;

                // Exact alignment with zero speed is the absorbing state we want.
                if(nx==tx && nv==0) score-=10000;

                // While already falling, prioritize staying inside the selected shaft;
                // before the fall, prioritize killing momentum at its exact top-left x.
                if(choiceDropStarted && nx==tx) score-=100;

                if(score<bestScore){
                    bestScore=score;
                    bestDir=dir;
                }
            }
            act.dir=bestDir;
            act.J=false;
        }

        if(isPuzzle){
            specialAct=true;
            bool grounded=screenRowSolid(27,21,26);
            bool plusNow=overlapPlusNow();
            if(puzzleLevelY==-1) puzzleLevelY=y;

            int pr=clamp((y+3)/6,0,N-1);
            int pc=clamp((x+3)/6,0,M-1);

            auto plusUnderOrInside=[&](){
                // At the very top of a + stack the body can be exactly one pixel/cell
                // above the last jump block, so overlapPlusNow() becomes false.  Keep
                // recognizing the shaft from the strip immediately below the body.
                for(int yy=y;yy<=min(H-1,y+6);yy++)
                    for(int xx=x;xx<=min(W-1,x+5);xx++)
                        if(insidePix(yy,xx) && world[yy][xx]=='+') return true;
                return false;
            };
            bool plusNear=plusNow || plusUnderOrInside();

            // Puzzle initial elevator controller.
            // Important generator property used here:
            //   * the initial + shaft is vertical,
            //   * horizontal rows open to the LEFT at specific heights,
            //   * some of those rows contain a coin farther away (possibly outside 48x48),
            //   * the top of the shaft itself also exits to the left.
            // Do NOT require a branch to be spike-free/flat: Puzzle rows intentionally
            // contain spikes, and rejecting them was why only the first branch was visited.
            auto solidRowWorld=[&](int yy,int xl,int xr){
                if(yy<0 || yy>=H) return true;
                xl=max(0,xl); xr=min(W-1,xr);
                for(int xx=xl;xx<=xr;xx++){
                    char ch=world[yy][xx];
                    if(ch=='#'||ch=='O'||ch=='?') return true;
                }
                return false;
            };
            auto bodyClearAt=[&](int py,int px){
                if(py<0||px<0||py+5>=H||px+5>=W) return false;
                for(int yy=py;yy<=py+5;yy++) for(int xx=px;xx<=px+5;xx++){
                    char ch=world[yy][xx];
                    if(ch=='#'||ch=='O'||ch=='?') return false;
                }
                return true;
            };
            auto predictVerticalY=[&](bool pressJ){
                int py=y, vv=motion.vy;
                bool canFresh = grounded || plusNear || (motion.prevJumpTime && !motion.prevJumped);
                bool canHold = motion.prevJumped && motion.hold>0;
                if(pressJ && (canFresh || canHold)) vv=-3;
                if(vv<0){
                    for(int k=0;k<-vv;k++){
                        int ny=py-1;
                        if(ny<0 || solidRowWorld(ny,x,x+5)) break;
                        py=ny;
                    }
                }else if(vv>0){
                    for(int k=0;k<vv;k++){
                        int by=py+6;
                        if(by>=H || solidRowWorld(by,x,x+5)) break;
                        ++py;
                    }
                }
                return py;
            };

            // A REAL Puzzle side lane is not just empty air to the left of the shaft.
            // It is a walkable horizontal corridor: the body can enter it and there is
            // floor/support underneath.  The previous test only checked empty space, so
            // the open air beside the elevator was mistaken for a branch at many y's,
            // producing the left-right-left-right oscillation even where no lane existed.
            auto supportedAt=[&](int py,int px){
                if(py<0 || px<0 || py+6>=H || px+5>=W) return false;
                int by=py+6;
                bool any=false;
                for(int xx=px;xx<=px+5;xx++){
                    char ch=world[by][xx];
                    if(ch=='?' ) return false;
                    if(ch=='#'||ch=='O'||ch=='+') any=true;
                }
                return any;
            };
            auto leftBranchOpenAt=[&](int py){
                if(puzzleShaftX<0) return false;
                // Check two consecutive body positions to the left.  This rejects a
                // one-cell pocket/air gap and recognizes the long Puzzle coin lanes.
                int p1=puzzleShaftX-6;
                int p2=puzzleShaftX-12;
                if(p2<0) return false;
                if(!bodyClearAt(py,p1) || !supportedAt(py,p1)) return false;
                if(!bodyClearAt(py,p2) || !supportedAt(py,p2)) return false;
                return true;
            };

            bool headBlocked=screenRowSolid(20,21,26);

            // IMPORTANT: screenColHas(20, ...) scans the ENTIRE 48px-tall local
            // column.  In Puzzle there are walls above/below almost every branch, so
            // that made a freshly entered left detour look blocked immediately and
            // flipped the FSM straight back to the elevator.  Collision decisions
            // must only inspect the 6px body edge at the player's actual y.
            auto bodyEdgeBlocked=[&](int dir){
                int ex=(dir<0?x-1:x+6);
                if(ex<0 || ex>=W) return true;
                for(int yy=y;yy<=y+5;yy++){
                    if(!insidePix(yy,ex)) return true;
                    char ch=world[yy][ex];
                    if(ch=='#'||ch=='O'||ch=='?') return true;
                }
                return false;
            };
            auto spikeImmediatelyAhead=[&](int dir){
                int xl=(dir<0?x-6:x+6);
                int xr=(dir<0?x-1:x+11);
                xl=max(0,xl); xr=min(W-1,xr);
                int yt=max(0,y-1), yb=min(H-1,y+6);
                for(int yy=yt;yy<=yb;yy++) for(int xx=xl;xx<=xr;xx++)
                    if(world[yy][xx]=='^') return true;
                return false;
            };
            bool leftBlocked=bodyEdgeBlocked(-1);
            bool rightBlocked=bodyEdgeBlocked(1);
            bool spikeLeft=spikeImmediatelyAhead(-1);

            // Find the ACTUAL logical x of the + column.  The old code latched the
            // player's current x as soon as the body merely overlapped '+'.  Because a
            // 6px body can overlap a 6px jump block while protruding several pixels to
            // the left, that made us ride the elevator off-centre and our head caught
            // the ceiling of every left branch.  The rightmost legal riding position
            // in this one-cell shaft is the jump-block cell itself: x = 6 * column.
            auto detectPuzzleShaftX=[&]()->int{
                int c0=max(0,(x-8)/6), c1=min(M-1,(x+13)/6);
                int best=-1,bestCnt=0;
                for(int c=c0;c<=c1;c++){
                    int xl=6*c, cnt=0;
                    for(int yy=max(0,y-18);yy<=min(H-1,y+24);yy++)
                        for(int xx=xl;xx<=min(W-1,xl+5);xx++)
                            if(world[yy][xx]=='+') cnt++;
                    // Prefer the denser vertical + column; on ties prefer the one to
                    // the right, which is exactly what the Puzzle initial elevator uses.
                    if(cnt>bestCnt || (cnt==bestCnt && cnt>0 && xl>best)){
                        bestCnt=cnt; best=xl;
                    }
                }
                return bestCnt>0?best:-1;
            };
            if(puzzleInitialAscent){
                int sx=detectPuzzleShaftX();
                if(sx!=-1) puzzleShaftX=sx;
            }
            bool nearInitialShaft=(puzzleInitialAscent && puzzleShaftX!=-1 &&
                                   abs(x-puzzleShaftX)<=14);

            // Exact horizontal controller used whenever we mount/re-enter the shaft.
            // We MUST reach (x=puzzleShaftX,vx=0) before issuing J.  In particular,
            // being one or two pixels left while still overlapping '+' is not enough.
            auto puzzleAlignDir=[&](int tx)->int{
                int bestDir=0;
                long long bestScore=(1LL<<60);
                for(int dir=-1;dir<=1;dir++){
                    int px=x, pv=motion.vx;
                    if(pv!=0 && (dir==0 || (pv>0?dir<0:dir>0)))
                        pv += (pv>0?-1:1);

                    if(dir!=0){
                        int edge=(dir<0?px-1:px+6);
                        bool blocked=(edge<0||edge>=W);
                        if(!blocked){
                            for(int yy=y;yy<=y+5;yy++){
                                char ch=world[yy][edge];
                                if(ch=='#'||ch=='O'||ch=='?'){ blocked=true; break; }
                            }
                        }
                        if(!blocked) pv=max(-4,min(4,pv+dir));
                    }

                    int step=(pv>0)-(pv<0);
                    for(int k=0;k<abs(pv);k++){
                        int edge=(step<0?px-1:px+6);
                        bool blocked=(edge<0||edge>=W);
                        if(!blocked){
                            for(int yy=y;yy<=y+5;yy++){
                                char ch=world[yy][edge];
                                if(ch=='#'||ch=='O'||ch=='?'){ blocked=true; break; }
                            }
                        }
                        if(blocked){ pv=0; break; }
                        px+=step;
                    }
                    long long score=120LL*llabs((long long)px-tx)+9LL*abs(pv);
                    if((x<tx&&px>tx)||(x>tx&&px<tx)) score+=45;
                    if(px==tx&&pv==0) score-=100000;
                    if(score<bestScore){ bestScore=score; bestDir=dir; }
                }
                return bestDir;
            };

            // At an intermediate opening the + column continues ABOVE us.  At the top
            // exit it does not.  This is much more reliable than waiting until a jump
            // physically hits the ceiling: the top corridor may have several pixels of
            // head room, causing an endless jump cycle.
            auto shaftHasPlusAbove=[&](int py){
                if(puzzleShaftX<0) return false;
                int xl=max(0,puzzleShaftX-1), xr=min(W-1,puzzleShaftX+6);
                for(int yy=max(0,py-24); yy<py; ++yy)
                    for(int xx=xl;xx<=xr;xx++)
                        if(world[yy][xx]=='+') return true;
                return false;
            };

            bool puzzleOverride=false;

            // --------------------------------------------------------------
            // HARD far-left drop transition.
            // Once a below-coin lane is selected, absolutely no Puzzle return, box,
            // elevator or snake-direction code may run until we have fallen and landed.
            // This fixes the old "reach left end -> turn right once -> miss the hole" bug.
            if(puzzleDropTarget!=-1){
                if(gotCoin) puzzleDropCollected=true;
                if((observedDy>0 && puzzleDropStartY!=-1 && y>puzzleDropStartY) || motion.vy>0)
                    puzzleDropStarted=true;

                // Finish only after a real descent and landing.  If the coin was picked
                // mid-fall, we still keep zero horizontal input until the landing.
                if(puzzleDropStarted && grounded && observedDy==0 &&
                   puzzleDropStartY!=-1 && y>=puzzleDropStartY+5){
                    bool collectedDropCoin=puzzleDropCollected ||
                        (puzzleDropTarget>=0 && coinAlive[puzzleDropTarget]!=1);
                    puzzleDropTarget=-1;
                    puzzleDropX=-1;
                    puzzleDropStartY=-1;
                    puzzleDropSince=-1;
                    puzzleDropStarted=false;
                    puzzleDropCollected=false;
                    puzzleDir=collectedDropCoin?1:-1;
                    puzzleCoinTarget=-1;
                    puzzleCoinSince=-1;
                    puzzleReturningCoin=false;
                    puzzleReturnDir=0;
                }else if(puzzleDropSince!=-1 && t-puzzleDropSince>220){
                    // Safety valve only; unlike the old code, timeout does not reverse
                    // before a long committed attempt.
                    puzzleDropTarget=-1;
                    puzzleDropX=-1;
                    puzzleDropStartY=-1;
                    puzzleDropSince=-1;
                    puzzleDropStarted=false;
                    puzzleDropCollected=false;
                }
            }

            // Arm the drop at a genuine left dead-end whenever a known mandatory coin
            // exists below.  This has priority even over a stale "returning coin" state.
            if(puzzleDropTarget==-1 && !puzzleInitialAscent && grounded && leftBlocked && puzzleDir<0){
                int bestCoin=-1,bestX=-1,bestScore=INT_MAX;

                auto canDropAt=[&](int tx){
                    if(tx<0 || tx+5>=W || y+6>=H) return false;
                    if(!bodyClearAt(y,tx)) return false;
                    for(int xx=tx;xx<=tx+5;xx++){
                        char ch=world[y+6][xx];
                        if(ch=='#'||ch=='O'||ch=='?') return false;
                    }
                    return true;
                };
                auto horizReachDrop=[&](int tx){
                    int cx=x,d=(tx>x)-(tx<x);
                    while(cx!=tx){
                        int nx=cx+d,edge=(d>0?nx+5:nx);
                        if(edge<0||edge>=W) return false;
                        for(int yy=y;yy<=y+5;yy++){
                            char ch=world[yy][edge];
                            if(ch=='#'||ch=='O'||ch=='?') return false;
                        }
                        cx=nx;
                    }
                    return true;
                };

                // The generator's left drop coins are close vertically but the opening
                // may be a few cells to the right of the literal wall.
                for(int rr=pr+1;rr<=min(N-1,pr+8);rr++){
                    for(int cc2=max(0,pc-4);cc2<=min(M-1,pc+14);cc2++){
                        int z=cid(rr,cc2);
                        if(coinAlive[z]!=1) continue;
                        int coinL=6*cc2+1, coinR=6*cc2+4;
                        int lo=max(0,x-6), hi=min(W-6,x+78);
                        for(int tx=lo;tx<=hi;tx++){
                            if(!canDropAt(tx) || !horizReachDrop(tx)) continue;
                            // Prefer a hole whose 6px body overlaps the coin column, then
                            // the nearest hole.  Do not arm a random unrelated pit.
                            bool overlap=(tx<=coinR && coinL<=tx+5);
                            int score=(overlap?0:800) + 12*abs((tx+3)-(6*cc2+3))
                                      + 70*(rr-pr) + abs(tx-x);
                            if(score<bestScore){
                                bestScore=score; bestCoin=z; bestX=tx;
                            }
                        }
                    }
                }

                if(bestCoin!=-1 && bestX!=-1 && bestScore<1200){
                    puzzleDropTarget=bestCoin;
                    puzzleDropX=bestX;
                    puzzleDropStartY=y;
                    puzzleDropSince=t;
                    puzzleDropStarted=false;
                    puzzleDropCollected=false;

                    // Kill every state that could order a right turn on the next frame.
                    puzzleCoinTarget=-1;
                    puzzleCoinSince=-1;
                    puzzleReturningCoin=false;
                    puzzleReturnDir=0;
                    puzzleLiftActive=false;
                    puzzleLiftStartY=-1;
                    puzzleLiftWasReturn=false;
                    puzzleLiftDetourExit=false;
                }
            }

            if(puzzleDropTarget!=-1){
                // Exact pre-drop alignment.  Once aligned, issue NO horizontal command
                // and NO jump; gravity is the only operation allowed to start the fall.
                if(x!=puzzleDropX || motion.vx!=0) act.dir=puzzleAlignDir(puzzleDropX);
                else act.dir=0;
                act.J=false;
                puzzleOverride=true;
            }

            if(!puzzleOverride && puzzleInitialAscent && puzzleInitialDetour==1){
                // HARD COMMIT to the selected side lane.  A one-frame leftBlocked sample
                // is not enough to turn around: while leaving the shaft our body can
                // temporarily brush the lip/ceiling of the branch.  Keep pushing LEFT
                // until either a coin is collected or x has genuinely stalled for several
                // consecutive observed frames after entering the lane.
                ++puzzleInitialDetourFrames;
                act.dir=-1;
                act.J=false;

                int entered=max(0,puzzleInitialDetourStartX-x);
                if(entered>=6) puzzleInitialDetourEntered=true;

                if(puzzleInitialDetourLastX==-1){
                    puzzleInitialDetourLastX=x;
                    puzzleInitialDetourStall=0;
                }else{
                    if(x < puzzleInitialDetourLastX){
                        puzzleInitialDetourStall=0;
                        puzzleInitialDetourLastX=x;
                    }else if(puzzleInitialDetourEntered){
                        ++puzzleInitialDetourStall;
                    }
                }

                // Once actually inside, jump only over a real spike.  Elevator J-hold
                // must never leak into the horizontal branch.
                if(puzzleInitialDetourEntered && spikeLeft &&
                   (grounded || motion.prevJumpTime))
                    act.J=true;

                bool branchFinished = gotCoin ||
                    (puzzleInitialDetourEntered && puzzleInitialDetourStall>=6);
                if(branchFinished){
                    if(0<=puzzleInitialDetourRow && puzzleInitialDetourRow<N)
                        puzzleInitialBranchDone[puzzleInitialDetourRow]=1;
                    puzzleInitialDetour=2;
                    puzzleInitialDetourFrames=0;
                    puzzleInitialDetourStall=0;
                    act.dir=1;
                    act.J=false;
                }else if(!puzzleInitialDetourEntered && puzzleInitialDetourFrames>90){
                    // False/poorly aligned mouth: give up only after a long attempt, and
                    // DO NOT mark it done so a later pass can retry it.
                    puzzleInitialDetour=2;
                    puzzleInitialDetourFrames=0;
                    puzzleInitialDetourStall=0;
                    act.dir=1;
                    act.J=false;
                }
                puzzleOverride=true;
            }else if(puzzleInitialAscent && puzzleInitialDetour==2){
                // Return to the TRUE right-hand shaft coordinate.  Merely overlapping
                // the jump block is insufficient: do not restart the elevator until
                // the whole 6px body is aligned to its cell and vx is exactly zero.
                if(puzzleShaftX==-1){
                    int sx=detectPuzzleShaftX();
                    if(sx!=-1) puzzleShaftX=sx;
                }
                if(puzzleShaftX!=-1 && (x!=puzzleShaftX || motion.vx!=0)){
                    act.dir=puzzleAlignDir(puzzleShaftX);
                    act.J=false;
                }else{
                    puzzleInitialDetour=0;
                    puzzleInitialDetourFrames=0;
                    puzzleInitialDetourStartX=-1;
                    puzzleInitialDetourRow=-1;
                    puzzleInitialDetourEntered=false;
                    puzzleInitialDetourLastX=-1;
                    puzzleInitialDetourStall=0;
                    act.dir=0;
                    act.J=true;
                    puzzleLiftActive=true;
                    puzzleLiftStartY=y;
                }
                puzzleOverride=true;
            }else if(nearInitialShaft){
                // Same rule on the initial mount and after any accidental partial
                // overlap: first slide fully to the right-hand + cell, then climb.
                if(puzzleShaftX!=-1 && (x!=puzzleShaftX || motion.vx!=0)){
                    act.dir=puzzleAlignDir(puzzleShaftX);
                    act.J=false;
                    puzzleOverride=true;
                }else{
                int pyNoJ=predictVerticalY(false);
                int pyJ=predictVerticalY(true);

                bool plusAboveNow=shaftHasPlusAbove(y);
                bool plusAboveNoJ=shaftHasPlusAbove(pyNoJ);
                bool plusAboveJ=shaftHasPlusAbove(pyJ);

                // TOP EXIT: we are still on/just above the initial + column, but there
                // is no + block above us anymore.  As soon as the left corridor is
                // physically enterable, leave the elevator permanently.  No ceiling
                // collision is required.
                bool topOpenNow=leftBranchOpenAt(y);
                bool topOpenNoJ=leftBranchOpenAt(pyNoJ);
                if((!plusAboveNow || !plusAboveNoJ) && (topOpenNow || topOpenNoJ)){
                    puzzleInitialAscent=false;
                    puzzleInitialDetour=0;
                    puzzleInitialDetourFrames=0;
                    puzzleInitialDetourStartX=-1;
                    puzzleInitialDetourRow=-1;
                    puzzleInitialDetourEntered=false;
                    puzzleInitialDetourLastX=-1;
                    puzzleInitialDetourStall=0;
                    puzzleLiftActive=false;
                    puzzleLiftStartY=-1;
                    puzzleLiftWasReturn=false;
                    puzzleLiftDetourExit=false;
                    puzzleDir=-1;
                    act.dir=-1;
                    act.J=false;
                    puzzleOverride=true;
                }else{
                    // INTERMEDIATE BRANCH: catch openings at the y where horizontal
                    // movement will actually happen this frame.  Prefer no-J if both
                    // choices align, so we do not overshoot the mouth vertically.
                    int branchY=-1;
                    bool branchPressJ=false;
                    if(plusAboveNoJ && leftBranchOpenAt(pyNoJ)){
                        branchY=pyNoJ;
                        branchPressJ=false;
                    }else if(plusAboveJ && leftBranchOpenAt(pyJ)){
                        branchY=pyJ;
                        branchPressJ=true;
                    }

                    int branchR=(branchY==-1?-1:clamp((branchY+3)/6,0,N-1));
                    if(branchY!=-1 && !puzzleInitialBranchDone[branchR]){
                        // Do not mark this row done merely because we SAW the mouth.
                        // Completion is recorded only after x proves that the body really
                        // left the elevator by at least one full cell.
                        puzzleInitialDetour=1;
                        puzzleInitialDetourFrames=0;
                        puzzleInitialDetourStartX=x;
                        puzzleInitialDetourRow=branchR;
                        puzzleInitialDetourEntered=false;
                        puzzleInitialDetourLastX=x;
                        puzzleInitialDetourStall=0;
                        act.dir=-1;
                        act.J=branchPressJ;
                        puzzleOverride=true;
                    }else{
                        // Stay vertically centred on the shaft until a real opening/top
                        // is encountered.
                        act.dir=0;
                        act.J=true;
                        if(!puzzleLiftActive){
                            puzzleLiftActive=true;
                            puzzleLiftStartY=y;
                            puzzleLiftWasReturn=false;
                        }
                        puzzleOverride=true;
                    }
                }
                } // exact shaft alignment completed before vertical controller
            }

            if(!puzzleOverride){
                // Finish a normal post-initial vertical transition only after we have
                // actually moved off its + column.  Merely becoming grounded while still
                // on the shaft is not a completed floor transition.
                if(puzzleLiftActive && grounded && y<=puzzleLiftStartY-5 && !plusNear){
                    if(puzzleLiftDetourExit){
                        puzzleLiftDetourExit=false;
                    }else if(puzzleLiftWasReturn){
                        puzzleReturningCoin=false;
                        puzzleReturnDir=0;
                            puzzleCoinTarget=-1;
                        puzzleCoinSince=-1;
                    }else{
                        puzzleDir=-puzzleDir;
                        puzzleCoinTarget=-1;
                        puzzleCoinSince=-1;
                        }
                    puzzleLevelY=y;
                    puzzleLiftActive=false;
                    puzzleLiftStartY=-1;
                    puzzleLiftWasReturn=false;
                }

                // Ordinary row coin lanes remain explicit detours after the initial
                // elevator phase.  These coins are already locally known when selected.
                if(gotCoin){
                    bool wasDetour=(puzzleCoinTarget!=-1);
                    puzzleCoinTarget=-1;
                    puzzleCoinSince=-1;
                    if(wasDetour){
                        puzzleReturningCoin=true;
                        puzzleReturnDir=-puzzleDir;
                        if(puzzleReturnDir==0) puzzleReturnDir=1;
                        }
                }
                if(puzzleCoinTarget!=-1 &&
                   (coinAlive[puzzleCoinTarget]!=1 ||
                    (puzzleCoinSince!=-1 && t-puzzleCoinSince>150))){
                    puzzleCoinTarget=-1;
                    puzzleCoinSince=-1;
                    puzzleLiftDetourExit=false;
                }

                // After the initial ascent, only select a concrete LEFT coin detour.
                // The initial shaft no longer depends on this visibility-limited code.
                if(puzzleCoinTarget==-1 && puzzleDir<0){
                    int best=-1,bestScore=INT_MAX;
                    for(int rr=max(0,pr-1);rr<=min(N-1,pr+4);rr++){
                        for(int cc2=max(0,pc-12);cc2<pc;cc2++){
                            int z=cid(rr,cc2);
                            if(coinAlive[z]!=1) continue;
                            int score=90*max(0,rr-pr)+3*(pc-cc2)+abs(rr-pr);
                            if(score<bestScore){ bestScore=score; best=z; }
                        }
                    }
                    if(best!=-1){
                        puzzleCoinTarget=best;
                        puzzleCoinSince=t;
                        }
                }

                // Find an untouched known box on the current horizontal band.  Every
                // encountered Puzzle box is a required checkpoint and must be pushed.
                int boxTarget=-1, boxTargetDist=INT_MAX;
                for(int c2=0;c2<M;c2++){
                    int z=cid(pr,c2);
                    if(!cellSeen[z] || puzzleBoxTouched[z]) continue;
                    int cy=6*pr+3, cx=6*c2+3;
                    if(!insidePix(cy,cx) || world[cy][cx]!='O') continue;
                    int d=abs(c2-pc);
                    if(d<boxTargetDist){ boxTargetDist=d; boxTarget=z; }
                }

                int desiredDir=puzzleDir;
                bool chasingCoin=(puzzleCoinTarget!=-1);
                if(puzzleReturningCoin){
                    desiredDir=puzzleReturnDir;
                }else if(chasingCoin){
                    int tc=puzzleCoinTarget%M;
                    int tx=6*tc;
                    if(x<tx) desiredDir=1;
                    else if(x>tx) desiredDir=-1;
                    else desiredDir=0;
                }else if(boxTarget!=-1){
                    int bc=boxTarget%M;
                    if(bc<pc) desiredDir=-1;
                    else if(bc>pc) desiredDir=1;
                }

                bool boxAhead=(desiredDir<0?screenColHas(20,'O'):
                               desiredDir>0?screenColHas(27,'O'):false);
                bool spikeAhead=(desiredDir<0?screenColHas(20,'^'):
                                 desiredDir>0?screenColHas(27,'^'):false);

                act.dir=desiredDir;

                if(boxAhead && grounded && desiredDir!=0){
                    int bc=clamp(pc+desiredDir,0,M-1);
                    int bz=cid(pr,bc);
                    puzzleBoxTouched[bz]=1;
                    act.J=false;
                }else if((plusNear || puzzleLiftActive) && headBlocked && !puzzleReturningCoin){
                    // Generic elevator top/landing exit.  Choose the physically open
                    // side; do not require direct overlap with '+'.
                    act.J=false;
                    if(!leftBlocked) act.dir=-1;
                    else if(!rightBlocked) act.dir=1;
                    else act.dir=0;
                }else if(motion.prevJumped && motion.hold>0){
                    act.J=true;
                }else if(puzzleReturningCoin){
                    act.J=(grounded||plusNear) && (plusNear || spikeAhead);
                }else if(chasingCoin){
                    int tr=puzzleCoinTarget/M;
                    if(tr>pr) act.J=spikeAhead && grounded;
                    else act.J=(grounded||plusNear) && (plusNear || spikeAhead);
                }else{
                    act.J=(grounded||plusNear) && (plusNear || spikeAhead);
                }

                if(plusNear && act.J && !puzzleLiftActive){
                    puzzleLiftActive=true;
                    puzzleLiftStartY=y;
                    puzzleLiftWasReturn=puzzleReturningCoin;
                    puzzleLiftDetourExit=false;
                }
            }
        }

        if(!specialAct){
        if(t<escapeUntil){
            // Anti-loop escape: commit long enough to actually leave the local basin.
            auto colHasScr=[&](int cc2,char a,char b){
                if(cc2<0||cc2>=48) return false;
                for(int rr=21;rr<=26;rr++) if(scr[rr][cc2]==a||scr[rr][cc2]==b) return true;
                return false;
            };
            auto rowHasScr=[&](int rr,int c1,int c2,char a,char b){
                if(rr<0||rr>=48) return false;
                c1=max(c1,0); c2=min(c2,47);
                for(int c=c1;c<=c2;c++) if(scr[rr][c]==a||scr[rr][c]==b) return true;
                return false;
            };
            bool groundedNow=rowHasScr(27,21,26,'#','O');
            bool plusNow=false;
            for(int gy=max(0,y);gy<=min(H-1,y+5)&&!plusNow;gy++)
                for(int gx=max(0,x);gx<=min(W-1,x+5);gx++)
                    if(world[gy][gx]=='+'){ plusNow=true; break; }

            bool wallAhead=(escapeDir>0?colHasScr(27,'#','O'):colHasScr(20,'#','O'));
            act.dir=escapeDir;
            if(motion.prevJumped && motion.hold>0) act.J=true;
            else act.J=(groundedNow||plusNow) && wallAhead;
        }else if(coarseGuidance){
            int pr=clamp((y+3)/6,0,N-1), pc=clamp((x+3)/6,0,M-1);
            int wantDx=(coarseNextC>pc)-(coarseNextC<pc);
            int wantDy=(coarseNextR>pr)-(coarseNextR<pr);
            if(wantDx!=0){
                act.dir=wantDx;
                // Never jump just because a box is ahead: clear it by repeated one-pixel pushes.
                act.J=false;
            }else if(wantDy<0){
                act.J=true;
            }else if(wantDy>0){
                int tx=6*coarseNextC;
                if(x<tx) act.dir=1;
                else if(x>tx) act.dir=-1;
                else if(motion.vx>0) act.dir=-1;
                else if(motion.vx<0) act.dir=1;
                else act.dir=0;
            }else{
                coarseGuidance=false;
            }
            // Re-evaluate after each frame because pushing changes the observed box map.
            coarseGuidance=false;
        }else if(!plan.empty()){
            act=plan.front();
            plan.pop_front();
        }else{
            // Last-resort recovery. Choose a known-map route toward the nearest frontier
            // instead of steering by raw Manhattan direction through walls.
            int pr=clamp((y+3)/6,0,N-1);
            int pc=clamp((x+3)/6,0,M-1);

            int nextR=pr,nextC=pc;
            bool haveCoarse=false;

            const int INF=1e9;
            vector<int> par(K,-1), dist(K,INF);
            using PI=pair<int,int>;
            priority_queue<PI,vector<PI>,greater<PI>> pq2;
            int st=cid(pr,pc);
            par[st]=st;
            dist[st]=0;
            pq2.push({0,st});
            int goal=-1;
            int bestGoalScore=INF;

            static const int dr[4]={0,0,-1,1};
            static const int dc[4]={-1,1,0,0};

            int fallbackPopped=0;
            const int fallbackPopLimit=(isJump && K>=6000 ? 420 : INT_MAX);
            while(!pq2.empty() && fallbackPopped<fallbackPopLimit){
                auto [cd,v]=pq2.top();pq2.pop();
                if(cd!=dist[v]) continue;
                ++fallbackPopped;
                int r=v/M,c=v%M;

                bool frontier=false;
                for(int k=0;k<4;k++){
                    int nr=r+dr[k],nc=c+dc[k];
                    if(insideCell(nr,nc) && !cellSeen[cid(nr,nc)]) frontier=true;
                }
                if(v!=st && frontier && badFrontierUntil[v]<=t){
                    int risk=cellTrapPenalty(r,c,pr);
                    int score=cd+risk;
                    if(score<bestGoalScore){
                        bestGoalScore=score;
                        goal=v;
                    }
                }

                for(int k=0;k<4;k++){
                    int nr=r+dr[k],nc=c+dc[k];
                    if(!insideCell(nr,nc)) continue;
                    int nv=cid(nr,nc);
                    if(!cellSeen[nv]) continue;
                    char ch=world[6*nr+3][6*nc+3];
                    if(ch=='#') continue;
                    if(ch=='O'){
                        // In fallback too, a box is only a horizontal push edge.
                        if(dr[k]!=0 || !coarsePushableBox(nr,nc,dc[k])) continue;
                    }

                    // Box clearing must dominate spike consumption.  A bare spike is
                    // still finite so the bot can cross it when literally unavoidable.
                    int w=10+min(60,getVisit(nv));
                    if(ch=='O') w+=(isPits?1:10);
                    if(ch=='^') w+=(shield>0?180:1800);
                    if(nr>r && world[6*nr+3][6*nc+3]!='+') w+=18;
                    int nd=cd+w;
                    if(nd<dist[nv]){
                        dist[nv]=nd;
                        par[nv]=v;
                        pq2.push({nd,nv});
                    }
                }
            }

            if(goal!=-1){
                if(fallbackGoal!=goal){
                    fallbackGoal=goal;
                    fallbackGoalSince=t;
                    fallbackGoalSeenCells=seenCells;
                }
                int v=goal;
                while(par[v]!=st && par[v]!=v) v=par[v];
                nextR=v/M; nextC=v%M;
                haveCoarse=true;
            }else{
                fallbackGoal=-1;
            }

            int wantDx=(nextC>pc)-(nextC<pc);
            int wantDy=(nextR>pr)-(nextR<pr);

            if(fallbackLock>0) fallbackLock--;
            else if(wantDx) fallbackDir=wantDx;

            if(xStall>12){
                fallbackDir=-fallbackDir;
                fallbackLock=24;
                xStall=0;
            }

            auto rowHasScr=[&](int rr,int c1,int c2,char a,char b){
                if(rr<0||rr>=48) return false;
                c1=max(c1,0); c2=min(c2,47);
                for(int c=c1;c<=c2;c++) if(scr[rr][c]==a||scr[rr][c]==b) return true;
                return false;
            };
            auto colHasScr=[&](int cc2,char a,char b){
                if(cc2<0||cc2>=48) return false;
                for(int rr=21;rr<=26;rr++) if(scr[rr][cc2]==a||scr[rr][cc2]==b) return true;
                return false;
            };

            bool groundedNow=rowHasScr(27,21,26,'#','O');
            bool plusNow=false;
            for(int gy=max(0,y);gy<=min(H-1,y+5)&&!plusNow;gy++)
                for(int gx=max(0,x);gx<=min(W-1,x+5);gx++)
                    if(world[gy][gx]=='+'){ plusNow=true; break; }

            bool jt=groundedNow||plusNow;

            // For a vertical drop, cell-level guidance is not enough: the whole 6-pixel
            // body must actually have empty space immediately below it.  Otherwise the old
            // controller could align to the logical column, stop, and wait forever while one
            // side of the body was still supported by a platform.
            if(haveCoarse && wantDy>0 && wantDx==0){
                auto canOccupyAtX=[&](int tx){
                    if(tx<0 || tx+5>=W) return false;
                    for(int yy=y;yy<=y+5;yy++){
                        for(int xx=tx;xx<=tx+5;xx++){
                            char ch=world[yy][xx];
                            if(ch=='#' || ch=='O' || ch=='?') return false;
                        }
                    }
                    return true;
                };

                auto canDropAtX=[&](int tx){
                    if(!canOccupyAtX(tx) || y+6>=H) return false;
                    for(int xx=tx;xx<=tx+5;xx++){
                        char ch=world[y+6][xx];
                        if(ch=='#' || ch=='O' || ch=='?') return false;
                    }
                    return true;
                };

                auto horizontalReachable=[&](int tx){
                    if(!canOccupyAtX(tx)) return false;
                    int cur=x;
                    int d=(tx>cur)-(tx<cur);
                    while(cur!=tx){
                        int nx=cur+d;
                        int edge=(d>0?nx+5:nx);
                        if(edge<0 || edge>=W) return false;
                        for(int yy=y;yy<=y+5;yy++){
                            char ch=world[yy][edge];
                            if(ch=='#' || ch=='O' || ch=='?') return false;
                        }
                        cur=nx;
                    }
                    return true;
                };

                int nominal=6*nextC;
                int tx=-1;

                // First try the exact logical column requested by the coarse path.
                if(canDropAtX(nominal) && horizontalReachable(nominal)){
                    tx=nominal;
                }else{
                    // Otherwise find the nearest REAL ledge/hole visible around us.
                    // Prefer positions close to the requested column, then close to us.
                    int lo=max(0,x-21), hi=min(W-6,x+21);
                    int bestScore=INT_MAX;
                    // Known coins below the ledge strongly bias the drop x: falling through
                    // a shaft is only useful if our 6px body actually overlaps the coin's 4px shape.
                    vector<pair<int,int>> belowCoins; // (cell row, cell col)
                    int curCellR=clamp((y+3)/6,0,N-1);
                    for(int z=0;z<K;z++) if(coinAlive[z]==1){
                        int rr=z/M, cc=z%M;
                        if(rr>curCellR && abs((6*cc+3)-(x+3))<=36) belowCoins.push_back({rr,cc});
                    }
                    auto coinBonus=[&](int cx){
                        int bonus=0;
                        for(auto [rr,cc]:belowCoins){
                            int L=6*cc+1, R=6*cc+4;
                            if(cx<=R && L<=cx+5){
                                int d=rr-curCellR;
                                bonus=max(bonus, max(0,900-12*d));
                            }
                        }
                        return bonus;
                    };
                    for(int cx=lo;cx<=hi;cx++){
                        if(!canDropAtX(cx) || !horizontalReachable(cx)) continue;
                        int score=3*abs(cx-nominal)+abs(cx-x)-coinBonus(cx);
                        if(score<bestScore){
                            bestScore=score;
                            tx=cx;
                        }
                    }
                }

                if(tx!=-1){
                    // Move to the real drop coordinate and kill horizontal momentum there.
                    if(x<tx){
                        if(motion.vx>0 && x+motion.vx>=tx) act.dir=-1;
                        else act.dir=1;
                    }else if(x>tx){
                        if(motion.vx<0 && x+motion.vx<=tx) act.dir=1;
                        else act.dir=-1;
                    }else{
                        if(motion.vx>0) act.dir=-1;
                        else if(motion.vx<0) act.dir=1;
                        else act.dir=0;
                    }
                    act.J=false;
                }else{
                    // The coarse "down" edge is physically impossible from here.  Do not
                    // freeze waiting for gravity; move along the platform and let the next
                    // frame re-evaluate the route.
                    int dir=fallbackDir;
                    bool wallAhead=(dir>0?colHasScr(27,'#','O'):colHasScr(20,'#','O'));
                    act.dir=dir;
                    act.J=jt && wallAhead;
                }
            }else{
                int dir = wantDx ? wantDx : fallbackDir;
                bool wallAhead=(dir>0?colHasScr(27,'#','O'):colHasScr(20,'#','O'));
                act.dir=dir;

                // Strong jump policy: after starting a normal jump, keep holding J
                // for both legal hold frames unless the exact plan explicitly says otherwise.
                if(motion.prevJumped && motion.hold>0){
                    act.J=true;
                }else{
                    act.J=jt && (wallAhead || wantDy<0);
                }

                // A jump block merely enables J; it is never a reason to jump by itself.
                if(plusNow && wantDy>=0 && !(motion.prevJumped && motion.hold>0))
                    act.J=false;
            }
        }

        } // !specialAct: generic navigator must not overwrite Jump/Puzzle FSM actions

        // Do not keep jumping in place after the judge has shown twice that the jump
        // produced no upward motion here.  Move sideways until we leave the local trap.
        if(t<jumpBanUntil && act.J && observedGroundedTop){
            act.J=false;
            if(act.dir==0) act.dir=jumpEscapeDir;
            plan.clear();
            coarseGuidance=false;
        }

        // Validate/predict.  The A* prefix snapshot may be stale, so if it rejects the
        // chosen action, simulate just ONE frame again from the latest observed world.
        // This is O(1) (at most a handful of pixels), unlike rebuilding the whole prefix map.
        bool hit=false;
        int haz=0;
        Motion nx;

        auto sideHasWorld=[&](int py,int px,int dir,char what){
            int xx=(dir<0?px-1:px+6);
            if(xx<0||xx>=W) return false;
            for(int yy=py;yy<=py+5;yy++) if(insidePix(yy,xx) && world[yy][xx]==what) return true;
            return false;
        };
        auto rowHasSolidWorld=[&](int yy,int xl,int xr){
            if(yy<0||yy>=H) return true;
            xl=max(xl,0); xr=min(xr,W-1);
            for(int xx=xl;xx<=xr;xx++){
                char ch=world[yy][xx];
                if(ch=='#'||ch=='O') return true;
            }
            return false;
        };
        auto colHasSolidWorld=[&](int xx,int yt,int yb){
            if(xx<0||xx>=W) return true;
            yt=max(yt,0); yb=min(yb,H-1);
            for(int yy=yt;yy<=yb;yy++){
                char ch=world[yy][xx];
                if(ch=='#'||ch=='O') return true;
            }
            return false;
        };
        auto jumpOverlapWorld=[&](int py,int px){
            for(int yy=py;yy<=py+5;yy++) for(int xx=px;xx<=px+5;xx++)
                if(insidePix(yy,xx) && world[yy][xx]=='+') return true;
            return false;
        };

        auto stepLatestWorld=[&](const Motion& in,Action a,Motion& out){
            Motion s=in;
            bool jt=rowHasSolidWorld(s.y+6,s.x,s.x+5) || jumpOverlapWorld(s.y,s.x);
            bool jumped=false;
            int nextHold=0;
            if(a.J){
                bool fresh=jt || (s.prevJumpTime && !s.prevJumped);
                if(fresh){ jumped=true; nextHold=2; }
                else if(s.prevJumped && s.hold>0){ jumped=true; nextHold=s.hold-1; }
            }
            if(jumped) s.vy=-3;

            if(s.vy<0){
                int cnt=-s.vy;
                for(int k=0;k<cnt;k++){
                    int ny=s.y-1;
                    if(ny<0 || rowHasSolidWorld(ny,s.x,s.x+5)){ s.vy=0; break; }
                    s.y=ny;
                }
            }else if(s.vy>0){
                int cnt=s.vy;
                for(int k=0;k<cnt;k++){
                    int by=s.y+6;
                    if(by>=H || rowHasSolidWorld(by,s.x,s.x+5)){ s.vy=0; break; }
                    s.y++;
                }
            }
            if(!rowHasSolidWorld(s.y+6,s.x,s.x+5) && !jumped) s.vy=min(6,s.vy+1);

            if(s.vx!=0 && (a.dir==0 || (s.vx>0?a.dir<0:a.dir>0)))
                s.vx += (s.vx>0?-1:1);

            if(a.dir!=0){
                bool wall=sideHasWorld(s.y,s.x,a.dir,'#');
                bool box=sideHasWorld(s.y,s.x,a.dir,'O');
                if(!wall && !box) s.vx=max(-4,min(4,s.vx+a.dir));
                // If a box is touching here, deliberatePush is handled outside.
                // Otherwise the command is ignored exactly like the judge.
            }

            if(s.vx<0){
                int cnt=-s.vx;
                for(int k=0;k<cnt;k++){
                    int nxp=s.x-1;
                    if(nxp<0 || colHasSolidWorld(nxp,s.y,s.y+5)){ s.vx=0; break; }
                    s.x=nxp;
                }
            }else if(s.vx>0){
                int cnt=s.vx;
                for(int k=0;k<cnt;k++){
                    int rx=s.x+6;
                    if(rx>=W || colHasSolidWorld(rx,s.y,s.y+5)){ s.vx=0; break; }
                    s.x++;
                }
            }

            s.prevJumpTime=jt;
            s.prevJumped=jumped;
            s.hold=jumped?nextHold:0;
            out=s;
            return true;
        };

        bool touchingBoxNow=(act.dir!=0 && sideHasWorld(motion.y,motion.x,act.dir,'O'));
        bool deliberatePush=(act.dir!=0 && !act.J && touchingBoxNow
                             && canPushSide(motion.y,motion.x,act.dir));
        if(deliberatePush){
            // Box motion is dynamic; trust the next observed frame instead of predicting it.
            havePrediction=false;
            plan.clear();
            coarseGuidance=false;
        }else{
            bool ok=stepState(motion,act,nx,-1,hit,haz);
            if(!ok){
                // Stale prefix only: recompute this one frame from the latest observed map.
                // Keep the chosen route/action instead of blindly wandering or freezing.
                stepLatestWorld(motion,act,nx);
            }
            predicted=nx;
            havePrediction=true;
        }

        lastIssued=act;
        haveLastIssued=true;
        emit(act);
    }
}
