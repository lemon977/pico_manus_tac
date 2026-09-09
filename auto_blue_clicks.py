"""Find bright blue LED blobs on glove/controller modules as click proxies for controller origin."""
import json, bisect, cv2, numpy as np
import calibrate_headcam as C
from make_review_video import load_aligned, find_sidecar, load_sidecar
from pico_controller_viz import _aligned_to_bundle, pose_to_T

def find_blue_blobs(bgr, min_area=8):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    # blue LED
    mask = cv2.inRange(hsv, (95, 80, 80), (140, 255, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3),np.uint8))
    cnts,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blobs=[]
    for c in cnts:
        a=cv2.contourArea(c)
        if a < min_area: continue
        M=cv2.moments(c)
        if M["m00"]<=0: continue
        cx=M["m10"]/M["m00"]; cy=M["m01"]/M["m00"]
        blobs.append((cx,cy,a))
    blobs.sort(key=lambda x:-x[2])
    return blobs

fr=load_aligned("logs/data/aligned/aligned_111.jsonl")
walls=[r["pico_wall_ns"] for r in fr]; w0=walls[0]
cap=C.open_video("logs/vst_111.h264"); fps=cap.get(cv2.CAP_PROP_FPS) or 25
nfr=int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0); nfr=0 if nfr<0 else nfr
vw=int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)); vh=int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
sc=load_sidecar(find_sidecar("logs/vst_111.h264")); sc=sc if sc.size else None

# sample every ~1.5s, keep frames with 2 clear blue blobs in lower/mid FOV
items=[]; debug=[]
os_makedirs= __import__("os").makedirs; os_makedirs("/tmp/blueann", exist_ok=True)
for t in np.arange(5, 90, 2.0):
    want=w0+int(t*1e9)
    j=bisect.bisect_left(walls,want); j=min(max(j,0),len(fr)-1)
    if j>0 and abs(walls[j-1]-want)<abs(walls[j]-want): j-=1
    b=_aligned_to_bundle(fr[j])
    if not b["head"] or not b["left"].get("pico_ref") or not b["right"].get("pico_ref"):
        continue
    f=C.get_frame(cap,sc,walls,w0,fps,nfr,0,vw//2,walls[j])
    if f is None: continue
    blobs=find_blue_blobs(f)
    # need at least 2, prefer ones in hand region (not too top)
    cand=[(x,y,a) for x,y,a in blobs if 30<x<1050 and 80<y<780]
    if len(cand)<2: continue
    # left = smaller x, right = larger x
    cand=sorted(cand[:6], key=lambda z:z[0])
    # take leftmost and rightmost among top-area blobs
    L=cand[0]; R=cand[-1]
    if R[0]-L[0] < 120:  # too close, probably same hand
        continue
    # also require both controllers have reasonable 3D separation
    Pl=pose_to_T(*b["left"]["pico_ref"])[:3,3]
    Pr=pose_to_T(*b["right"]["pico_ref"])[:3,3]
    if np.linalg.norm(Pl-Pr)<0.08: continue
    items.append({"idx":int(j),"left":[int(round(L[0])),int(round(L[1]))],
                  "right":[int(round(R[0])),int(round(R[1]))], "t":round(t,1)})
    # debug image
    vis=f.copy()
    cv2.circle(vis,(int(L[0]),int(L[1])),10,(255,220,0),2); cv2.putText(vis,"L",(int(L[0])+8,int(L[1])),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,220,0),2)
    cv2.circle(vis,(int(R[0]),int(R[1])),10,(0,165,255),2); cv2.putText(vis,"R",(int(R[0])+8,int(R[1])),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,165,255),2)
    cv2.putText(vis,"t=%.1f idx=%d"%(t,j),(10,vis.shape[0]-10),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,0),2)
    cv2.imwrite("/tmp/blueann/t%05.1f.png"%t, vis)
    if len(items)>=16: break

# diversify: keep spaced indices
picked=[]
for it in items:
    if not picked or it["idx"]-picked[-1]["idx"]>=180:
        picked.append(it)
print("raw",len(items),"picked",len(picked))
for it in picked: print(it)
out={"eye":"left","note":"blue LED blob centers as controller/module proxies","items":[
    {"idx":it["idx"],"left":it["left"],"right":it["right"]} for it in picked]}
json.dump(out, open("clicks_111.json","w"), indent=2)
print("wrote clicks_111.json")
cap.release()
