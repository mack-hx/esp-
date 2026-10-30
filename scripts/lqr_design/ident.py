import csv, sys, numpy as np
def identify(f,LEG):
    R=list(csv.DictReader(open(f))); seen={}
    for r in R: seen[r["t_ms"]]=r
    R=sorted(seen.values(),key=lambda r:float(r["t_ms"])); g=lambda k:np.array([float(r[k]) for r in R])
    t=g("t_ms")/1000; leg=g("kd"); st=g("st"); ex=g("yaw")
    idx=np.where((st==2)&(ex!=0)&(leg==LEG))[0]
    runs=np.split(idx,np.where(np.diff(idx)!=1)[0]+1); run=max(runs,key=lambda r:t[r[-1]]-t[r[0]])
    i0,i1=run[0],run[-1]+1; T0=t[i0]; K=int((t[i1-1]-T0)/0.01)+1; kk=np.rint((t[i0:i1]-T0)/0.01).astype(int)
    def grid(y,hold=False):
        G=np.full(K,np.nan); G[np.clip(kk,0,K-1)]=y[i0:i1]
        if hold:
            for k in range(1,K):
                if np.isnan(G[k]): G[k]=G[k-1]
            return G
        n=np.isnan(G); ii=np.arange(K); G[n]=np.interp(ii[n],ii[~n],G[~n]); return G
    P=grid(g("pitch")); V=grid(g("pos_vel")); O=grid(g("out")); E=grid(g("yaw"),True)
    u=np.trunc(np.clip(O+E,-60,60)); e=np.where(u==0,0.0,np.sign(u)*(np.abs(u)+1.8))
    L=256; step=128; w=np.hanning(L); fr=np.fft.rfftfreq(L,0.01)
    def cs(a,b):
        acc=0
        for s in range(0,K-L,step):
            A=np.fft.rfft((a[s:s+L]-a[s:s+L].mean())*w); B=np.fft.rfft((b[s:s+L]-b[s:s+L].mean())*w); acc=acc+np.conj(A)*B
        return acc
    See=cs(E,e); GV=cs(E,V)/See; GP=cs(E,P)/See
    coh=np.abs(cs(E,e))**2/(cs(E,E).real*cs(e,e).real); sel=(fr>0.5)&(fr<12); wv=2j*np.pi*fr
    def fit(Td):
        D=np.exp(-wv*Td); rows=[];rhs=[]
        for i in np.where(sel)[0]:
            s=wv[i]; hv=GV[i]; hp=GP[i]; d=D[i]; ho=s*hp; wt=np.sqrt(coh[i])
            for r,y in ((np.array([hv,hp,ho,0,0,0,d,0])*wt,s*hv*wt),(np.array([0,0,0,hv,hp,ho,0,d])*wt,s*ho*wt)):
                rows.append(np.concatenate([r.real,r.imag]).reshape(2,-1)); rhs.append([y.real,y.imag])
        M=np.vstack(rows); b=np.array(rhs).reshape(-1); c=np.linalg.lstsq(M,b,rcond=None)[0]
        return c,np.sum((M@c-b)**2)/np.sum(b**2)
    best=None
    for Td in np.arange(0.0,0.081,0.005):
        c,r=fit(Td)
        if best is None or r<best[2]: best=(Td,c,r)
    return best[1],int(round(best[0]*1000)),best[2],coh[sel].mean(),(T0-t[0],t[i1-1]-t[0])
if __name__=="__main__":
    D="/Volumes/Moving/work/01_code/03_Personal/ESP32/esp_balance_rob/logs/data/"
    for f,leg in (("id_leg40_amp15_handsoff_2026-10-30.csv",40),("id_leg70_amp15_handsoff_2026-10-30.csv",70),("id_leg100_amp15_handsoff_2026-10-30.csv",100)):
        c,nd,r,coh,seg=identify(D+f,leg); print(leg,"delay",nd,"resid %.3f coh %.2f seg %.1f..%.1f"%(r,coh,*seg),"c=",[round(float(x),3) for x in c])
