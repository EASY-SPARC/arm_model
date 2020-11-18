#!/usr/bin/env python2
import numpy as np

class Robot:
    def __init__(self, joint_states, link_dimensions, M, D, K):
        """
        Parametros de entrada:
            M: float()              ### Escalar representando a matriz de massa do atuador robotico;
            D: np.matrix((3,3))     ### Matriz de amortecimento(damping) referente ao amortecedor;
            K: np.matrix((3,3))     ### Matriz de elasticidade referente a mola;
        """
        self.theta = joint_states
        [self.d1, self.l2, self.l3, self.d4]  = link_dimensions
        self.M = M
        self.D = D
        self.K = K

        self.J = 0

    def setJointStates(self, joint_states):
        self.theta = joint_states
    def getJointStates(self):
        return self.theta

    def jacobianComputation(self):
        #Parametros Recorrentes
        d4s1s234 = self.d4*np.sin(self.theta[0])*np.sin(self.theta[1]+self.theta[2]+self.theta[3])
        d4s1c234 = self.d4*np.sin(self.theta[0])*np.cos(self.theta[1]+self.theta[2]+self.theta[3])
        d4c1s234 = self.d4*np.cos(self.theta[0])*np.sin(self.theta[1]+self.theta[2]+self.theta[3])
        d4c1c234 = self.d4*np.cos(self.theta[0])*np.cos(self.theta[1]+self.theta[2]+self.theta[3])

        l3s1s23 = self.l3*np.sin(self.theta[0])*np.sin(self.theta[1]+self.theta[2])
        l3s1c23 = self.l3*np.sin(self.theta[0])*np.cos(self.theta[1]+self.theta[2])
        l3c1s23 = self.l3*np.cos(self.theta[0])*np.sin(self.theta[1]+self.theta[2])
        l3c1c23 = self.l3*np.cos(self.theta[0])*np.cos(self.theta[1]+self.theta[2])

        l2s1s2 = self.l2*np.sin(self.theta[0])*np.sin(self.theta[1]+self.theta[2])
        l2s1c2 = self.l2*np.sin(self.theta[0])*np.cos(self.theta[1]+self.theta[2])
        l2c1s2 = self.l2*np.cos(self.theta[0])*np.sin(self.theta[1]+self.theta[2])
        l2c1c2 = self.l2*np.cos(self.theta[0])*np.cos(self.theta[1]+self.theta[2])

        d4s234 = self.d4*np.sin(self.theta[1]+self.theta[2]+self.theta[3])
        l3s23 = self.l3*np.sin(self.theta[1]+self.theta[2])
        l2s2 = self.l2*np.sin(self.theta[2])
        s1 = np.sin(self.theta[0])
        c1 = np.cos(self.theta[0])
        
        J = np.array([[d4s1s234+l3s1s23+l2s1s2,   d4c1c234+l3c1c23+l2c1c2,              d4c1c234+l3c1c23,               d4c1c234,               ],\
                      [-d4c1s234-l3c1s23-l2c1s2,  -d4s1c234-l3s1c23-l2s1c2,             -d4s1c234-l3s1c23,              -d4s1c234,              ],\
                      [0.0,                       (c1**2-s1**2)*(d4s234+l3s23+l2s2),    (c1**2-s1**2)*(d4s234+l3s23),   (c1**2-s1**2)*d4s234,   ],\
                      [0.0,                       s1,                                   s1,                             s1,                     ],\
                      [0.0,                       c1,                                   c1,                             c1,                     ],\
                      [1.0,                       0.0,                                  0.0,                            0.0,                    ]])

        return J

    def admitance_control(self, f, M, D, K, p, v, Ts):
        """
        Parametros de entrada:
            f: np.array([x,y,z]).T  ### A forca exercida por aquele que desempenha o papel de mestre;
                                    # x,y,z representam as componentes da mesma sobre os respectivos eixos;
            p: np.array([x,y,z]).T  ### A posicao do efetuador no espaco;
                                    # x,y,z representam as componentes da mesma sobre os respectivos eixos;
            v: np.array([x,y,z]).T  ### A velocidade do efetuador no espaco;
                                    # x,y,z representam as componentes da mesma sobre os respectivos eixos;
            Ts: float()             ### O tempo de amostragem.

        """
        a = (f - D*v - K*p)/M
        v = v + a*Ts
        p = p + v*Ts

        return a, v, p