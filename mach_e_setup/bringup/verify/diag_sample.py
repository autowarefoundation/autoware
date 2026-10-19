import rclpy, collections, time, sys
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from tier4_system_msgs.msg import DiagGraphStatus, DiagGraphStruct, OperationModeAvailability
from autoware_adapi_v1_msgs.msg import MrmState
rclpy.init(); n=Node('claude_diag_sample')
S={'nodes':None,'diags':None}; cnt=collections.Counter(); msgs={}; av=collections.Counter(); mrm=collections.Counter(); N=[0]
def st(m): S['nodes']=[x.path for x in m.nodes]; S['diags']=[(x.parent,x.name) for x in m.diags]
def status(m):
    if S['nodes'] is None: return
    N[0]+=1
    for i,x in enumerate(m.nodes):
        if ord(x.level)>=1: cnt['NODE '+S['nodes'][i]]+=1
    for i,x in enumerate(m.diags):
        if ord(x.level)>=1:
            k='DIAG '+S['diags'][i][1]; cnt[k]+=1; msgs[k]=x.message[:120]
n.create_subscription(DiagGraphStruct,'/diagnostics_graph/struct',st,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
n.create_subscription(DiagGraphStatus,'/diagnostics_graph/status',status,10)
n.create_subscription(OperationModeAvailability,'/system/operation_mode/availability',lambda m:av.update([m.autonomous]),10)
n.create_subscription(MrmState,'/system/fail_safe/mrm_state',lambda m:mrm.update([(m.state,m.behavior)]),10)
t=time.time(); D=float(sys.argv[1])
while time.time()-t<D: rclpy.spin_once(n,timeout_sec=0.05)
print('status msgs',N[0],'autonomous available:',dict(av),'mrm:',dict(mrm))
for k,v in cnt.most_common(30): print(v,k,'|',msgs.get(k,''))
