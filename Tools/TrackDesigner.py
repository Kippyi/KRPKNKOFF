# Track designer: same closed uniform Catmull-Rom + 0.75 m resampling as AKartTestTrackActor.
import sys, json, numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
def cr(p0,p1,p2,p3,t):
    t2=t*t; t3=t2*t
    return 0.5*((2*p1)+(-p0+p2)*t+(2*p0-5*p1+4*p2-p3)*t2+(-p0+3*p1-3*p2+p3)*t3)
def build(ctl, spacing=0.75):
    c=np.array([q[:2] for q in ctl],float); n=len(c); dense=[]
    for i in range(n):
        for k in range(200): dense.append(cr(c[i-1],c[i],c[(i+1)%n],c[(i+2)%n],k/200))
    dense=np.array(dense); S=[dense[0]]; acc=0; L=0
    for i in range(1,len(dense)+1):
        p=dense[i%len(dense)]; d=np.linalg.norm(p-dense[i-1]); acc+=d; L+=d
        if acc>=spacing and i<len(dense): S.append(p); acc=0
    return np.array(S), L
def analyse(ctl, name, width=8.0):
    S,L=build(ctl); n=len(S)
    T=np.roll(S,-1,0)-np.roll(S,1,0); T/=np.linalg.norm(T,axis=1)[:,None]
    T0=np.roll(T,1,0); T1=np.roll(T,-1,0); ds=np.linalg.norm(np.roll(S,-1,0)-np.roll(S,1,0),axis=1)
    k=(T0[:,0]*T1[:,1]-T0[:,1]*T1[:,0])/ds
    ks=np.convolve(np.concatenate([k[-4:],k,k[:4]]),np.ones(9)/9,'valid')
    rmin=1/np.max(np.abs(ks)); imin=np.argmax(np.abs(ks))
    # clearance: min distance between points > 60 m apart along the track
    s=np.arange(n)*L/n; best=1e9; bi=(0,0)
    for i in range(0,n,2):
        d=np.linalg.norm(S-S[i],axis=1); sep=np.abs(s-s[i]); sep=np.minimum(sep,L-sep)
        d[sep<60]=1e9; j=np.argmin(d)
        if d[j]<best: best=d[j]; bi=(i,j)
    straight=0; cur=0
    for v in np.abs(ks):
        cur = cur+L/n if v<1/150 else 0; straight=max(straight,cur)
    print(f"{name}: length {L:.0f} m, tightest radius {rmin:.1f} m at s={s[imin]:.0f}, min clearance {best:.1f} m (s={s[bi[0]]:.0f} vs {s[bi[1]]:.0f}), longest straight {straight:.0f} m, corners(r<40) {np.sum(np.diff((np.abs(ks)>1/40).astype(int))==1)}")
    fig,ax=plt.subplots(figsize=(9,9)); N=np.stack([-T[:,1],T[:,0]],1)
    for side in (1,-1):
        E=S+N*side*width/2; ax.plot(np.r_[E[:,0],E[0,0]],np.r_[E[:,1],E[0,1]],'k',lw=1)
    kerb=np.abs(ks)>1/40; ax.scatter(S[kerb,0],S[kerb,1],s=2,c='r')
    ax.plot(S[:,0],S[:,1],'0.6',lw=0.5)
    for i,q in enumerate(ctl):
        cam=q[2] if len(q)>2 else 0
        ax.plot(q[0],q[1],'o',ms=4,color='b' if cam==0 else ('orange' if cam>0 else 'purple')); ax.annotate(str(i),(q[0],q[1]),fontsize=7,color='b')
    for m in range(0,int(L),100):
        i=int(m/L*n); ax.annotate(f"{m}",S[i],fontsize=8,color='g')
    ax.annotate("S/F",S[0],color='m',fontsize=10,weight='bold'); ax.arrow(S[0,0],S[0,1],T[0,0]*15,T[0,1]*15,width=1.5,color='m')
    ax.set_aspect('equal'); ax.grid(alpha=0.3); ax.set_title(f"{name}  {L:.0f} m  rmin {rmin:.1f} m  clearance {best:.1f} m")
    fig.savefig(f"{name}.png",dpi=80); plt.close(fig)
if __name__=="__main__":
    spec=json.load(open(sys.argv[1])); analyse(spec["controlPoints"], spec["name"])
