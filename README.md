# Matrix_EulerAngles_Quaternion
3D游戏数学基础  
方位：描述物体朝向(向量是没有方位的)  
角位移：方位通过相对于已知方位的旋转来描述，旋转的量称为角位移。  

欧拉角：让物体开始于标准方位( 物体坐标轴和惯性坐标轴对齐 )，在标准方位上，让物体做heading,pitch,bank旋转。  
heading-pitch-bank定义了向量从惯性坐标系转换到物体坐标系的旋转顺序  

四元数：  
欧拉证明了一个旋转序列等价于单个旋转=》因此3D中的任意角位移都能表示为绕单一轴的单一旋转  
旋转轴：( nx, ny, nz )  轴-角对（n, theta）定义了一个角位移: 绕轴n旋转theta角  
q = [ cos(theta/2)  (sin(theta/2)nx)   (sin(theta/2)ny)    (sin(theta/2)nz) ]  
q与-q表示的实际角位移相同的( theta增加360度 )  

如何从四元数获取旋转轴与旋转角：  
从定义式可推： theta = 2*acos(w)  
nx = q.x/sin(theta/2) ny=q.y/sin(theta/2) nz=q.z/sin(theta/2)  

四元数共轭：  
q* = [ w, -x, -y, -z ]  
可以看成：  
1.旋转轴方向与q相反，旋转角度相同  
2.旋转角度与q相反，旋转轴方向相同  

四元数的逆：
q-1 = q*/|| q ||

使用四元数使点p绕n旋转：
1.3D点p（x,y,z）扩展到齐次坐标p（0,x, y, z）
2.执行p' = qpq-1  
  
四元数乘法连接多次旋转：  
(四元数乘积的逆等于各个四元数的逆以相反的顺序相乘)  
p' = b(apa-1)b-1  
    =(ba)p(ba)-1  
因此先进行a旋转再进行b旋转等价于执行一次ba旋转，因此四元数乘法能连接多次旋转。  

标准四元数乘法定义为p' = qpq-1  
3D游戏数学基础一书定义的四元数乘法：  
p' = q-1pq
将四元数放在向量右边，四元数的逆放在左边。这样连接多次旋转为：
p' = b-1a-1pab = (ab)-1p(ab)
这样表达式自左向右，与旋转顺序相同，先a后b

四元数的差：
给定方位a,和b，计算从a旋转到b的角位移d, 即已知：ad = b求d  
d = a-1b  
上面的方位a,b应该可以认为就是角位移,或者从欧拉角，矩阵获取方位  

-----------------------------------------------------------------  
alpha = theta/2 ||n ||=1 
q=[ cos(alpha)  nsin(alpha) ]=[ cos(alpha) x*sin(alpha) y*sin(alpha) z*sin(alpha) ]  
四元数对数： 
logq = log[ cos(alpha)  n*sin(alpha) ]  
=[ 0 alpha*n ]  
四元数求指数：  
expp = exp([0 alpha*n ]) = [ cos(alpha) n*sin(alpha) ]  
四元数的指数总是返回单位四元数  
四元数求幂：  
qt = exp( t*log(q) )   
注意：四元数表达角位移时使用的都是最短圆弧，不能绕圈。  
因此(q4)1/2不是q2  
对数运算：提取轴n和角度theta, 和指数t进行标量乘结果是theta*t, 最后指数运算“撤销了”对数运算，从t*theta和n重新计算w, v。  
简单的说就是theta变为theta*t，旋转轴n不变，从轴角对n-theta*t计算四元数  
另外，单位四元数的任意次方还是单位四元数  
