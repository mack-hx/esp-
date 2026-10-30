import numpy as np, itertools
MODELS={"70":([-16.439,-10.511,2.54,-95.468,139.374,-16.808,-0.661,-61.395],15),
        "40":([-14.72,-11.25,2.53,-14.26,105.57,-33.71,-0.94,-65.78],20),
        "100":([-15.21,-12.85,2.12,-93.57,158.05,-20.87,-0.32,-79.74],15)}
LEAN=5.0; UMAX=60.0
def expm(M):
    E=np.eye(len(M)); T=np.eye(len(M))
    for k in range(1,40): T=T@M/k; E=E+T
    return E
def disc(c,sB=1.0):
    avv,avp,avo,aov,aop,aoo,bv,bo=c
    A=np.array([[0,1,0,0],[0,avv,avp,avo],[0,0,0,1],[0,aov,aop,aoo]],float); B=np.array([0,bv,0,bo])*sB
    M=np.zeros((5,5)); M[:4,:4]=A*0.001; M[:4,4]=B*0.001; E=expm(M); return E[:4,:4],E[:4,4]
def law(K,x):
    kx,kv,kp,kd=K; lean=(kx*x[0]+kv*x[1])/kp
    return float(np.clip(kp*x[2]+kd*x[3]+kp*LEAN*np.tanh(lean/LEAN),-UMAX,UMAX))
def sim(Ad,Bd,K,nd,x0,T=5.0):
    x=np.array(x0,float); buf=[0.0]*nd; pk=0; pu=0
    tr=[]
    for k in range(int(T/0.01)):
        u=law(K,x)
        for j in range(10): x=Ad@x+Bd*buf[0]; buf=buf[1:]+[u]
        tr.append([x[0],x[2],u])
        if abs(x[2])>30 or abs(x[0])>200: return None
    return np.array(tr)
def stl(a,thr):
    i=np.where(np.abs(a)>thr)[0]; return (i[-1]+1)*0.01 if len(i) else 0.0
def evalK(K):
    out={}
    for name,(c,nd) in MODELS.items():
        Ad,Bd=disc(c); t1=sim(Ad,Bd,K,nd,[0,0,3,0]); t2=sim(Ad,Bd,K,nd,[6,0,0,0])
        if t1 is None or t2 is None: return None
        rob=0
        for sB,dn in ((0.7,0),(1.3,0),(1.0,-5),(1.0,5),(1.0,10),(0.8,5)):
            a,b=disc(c,sB); t=sim(a,b,K,max(1,nd+dn),[0,0,3,0],T=6.0)
            rob+=bool(t is not None and np.abs(t[-100:,0]).max()<0.5 and np.abs(t[-100:,1]).max()<0.3)
        out[name]=dict(sP=stl(t1[:,1],0.3),pU=np.abs(t1[:,2]).max(),sX=stl(t2[:,0],0.5),pP=np.abs(t2[:,1]).max(),rob=rob)
    return out
def reduced(Ad,Bd):
    g2=sum(np.linalg.matrix_power(Ad,9-j)@Bd for j in range(5)); g1=sum(np.linalg.matrix_power(Ad,9-j)@Bd for j in range(5,10))
    A=np.zeros((6,6)); A[:4,:4]=np.linalg.matrix_power(Ad,10); A[:4,4]=g1; A[:4,5]=g2; A[5,4]=1
    B=np.zeros((6,1)); B[4,0]=1; return A,B
def dlqr(A,B,Q,R):
    P=Q.copy()
    for _ in range(6000):
        Kp=np.linalg.solve(R+B.T@P@B,B.T@P@A); Pn=A.T@P@(A-B@Kp)+Q
        if np.max(np.abs(Pn-P))<1e-9: P=Pn;break
        P=Pn
    return -np.linalg.solve(R+B.T@P@B,B.T@P@A)[0]
def score(r): return max(v["sP"]+v["sX"] for v in r.values()) if max(v["pP"] for v in r.values())<=6.5 else 99
def show(tag,K,r):
    if r is None: print(tag,"UNSTABLE"); return
    print("%-26s K=%s robust %s | "%(tag,np.round(K,2),[v["rob"] for v in r.values()])+" | ".join("%s: P%.2fs X%.2fs pk%.1f U%.0f"%(n,v["sP"],v["sX"],v["pP"],v["pU"]) for n,v in r.items()))
base=np.array([-12*1.0,-12*0.3,12.0,0.4]); show("old PID 12/.4/1/.3",base,evalK(base))
q=np.array([-13*1.27,-13*0.18,13.0,0.4]); show("quantized 13/.4/1.27/.18",q,evalK(q))
cands=[]
for mname,(c,nd) in MODELS.items():
    Ad,Bd=disc(c); Ar,Br=reduced(Ad,Bd)
    for xw,vw,ow,Rv in itertools.product((30,100,300),(0.3,1),(0.03,0.1),(0.03,0.1,0.3,1.0,3.0)):
        K=dlqr(Ar,Br,np.diag([xw,vw,1.0,ow,0,0]),np.array([[Rv]]))[:4]
        if not(K[2]>0 and K[0]<0 and K[1]<0 and K[3]>0): continue
        r=evalK(K)
        if r is None or any(v["rob"]<6 for v in r.values()): continue
        cands.append((score(r),mname,(xw,vw,ow,Rv),K,r))
cands.sort(key=lambda z:z[0]); print("designs robust on all 3 models:",len(cands))
for s,m,w,K,r in cands[:4]: show("score %.2f %s %s"%(s,m,w),K,r)
