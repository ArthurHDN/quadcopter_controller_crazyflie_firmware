from quaternion import *

import unittest

class TestQuaternionMethods(unittest.TestCase):

    def test_eq_quat(self):
        q = Quaternion([1,2,3,4])
        q1 = Quaternion([1,2,3,4])
        h = Quaternion([5,6,7,8])
        self.assertTrue(q == q1)
        self.assertFalse(q == h)
        self.assertFalse(q != q1)
        self.assertTrue(q != h)

    def test_real(self):
        re = 10
        q = Quaternion([re,2,3,4])
        self.assertEqual(q.real, re)

    def test_imag(self):
        im = np.array([1.,2.,3.])
        q = Quaternion([1,im[0], im[1], im[2]])
        self.assertIsInstance(q.imag, type(im) )
        self.assertTrue( (q.imag == im).all() )

    def test_norm(self):
        q_data = [1,2,3,4]
        q = Quaternion(q_data)
        q_norm = (q_data[0]**2+q_data[1]**2+q_data[2]**2+q_data[3]**2)**(1/2)
        self.assertLess(q.norm - q_norm, 0.001)
        self.assertGreater(q.norm - q_norm, -0.001)

    def test_conj(self):
        q = Quaternion([1,2,3,4])
        q_conj = Quaternion([1,-2,-3,-4])
        self.assertEqual(q.conj(), q_conj)

    def test_sum_quat(self):
        q1 = Quaternion([1,2,3,4])
        q2 = Quaternion([5,6,7,8])
        q1_add_q2 = Quaternion([1+5,2+6,3+7,4+8])
        q2_add_q1 = q1_add_q2
        self.assertEqual(q1+q2, q1_add_q2)
        self.assertEqual(q2+q1, q2_add_q1)
        q1_sub_q2 = Quaternion([1-5,2-6,3-7,4-8])
        q2_sub_q1 = -q1_sub_q2
        self.assertEqual(q1-q2, q1_sub_q2)
        self.assertEqual(q2-q1, q2_sub_q1)

    def test_sum_int(self):
        x = 10
        q = Quaternion([1,2,3,4])
        q_add_x = Quaternion([1+x,2,3,4])
        x_add_q = q_add_x
        self.assertEqual(q+x, q_add_x)
        self.assertEqual(x+q, x_add_q)
        q_sub_x = Quaternion([1-x,2,3,4])
        x_sub_q = -q_sub_x
        self.assertEqual(q-x, q_sub_x)
        self.assertEqual(x-q, x_sub_q)

    def test_sum_float(self):
        x = 5.12
        q = Quaternion([1,2,3,4])
        q_add_x = Quaternion([1+x,2,3,4])
        x_add_q = q_add_x
        self.assertEqual(q+x, q_add_x)
        self.assertEqual(x+q, x_add_q)
        q_sub_x = Quaternion([1-x,2,3,4])
        x_sub_q = -q_sub_x
        self.assertEqual(q-x, q_sub_x)
        self.assertEqual(x-q, x_sub_q)

    def test_mul_quat(self):
        q1 = Quaternion([1,2,3,4])
        q2 = Quaternion([5,6,7,8])
        q1_mul_q2 = Quaternion([-60,12,30,24])
        q2_mul_q1 = Quaternion([-60,20,14,32])
        self.assertEqual(q1*q2, q1_mul_q2)
        self.assertEqual(q2*q1, q2_mul_q1)

    def test_mul_int(self):
        x = 15
        q = Quaternion([1,2,3,4])
        q_mul_x = Quaternion([x*1,x*2,x*3,x*4])
        x_mul_q = q_mul_x
        self.assertEqual(q*x, q_mul_x)
        self.assertEqual(x*q, x_mul_q)

    def test_mul_float(self):
        x = 7.8
        q = Quaternion([1,2,3,4])
        q_mul_x = Quaternion([x*1,x*2,x*3,x*4])
        x_mul_q = q_mul_x
        self.assertEqual(q*x, q_mul_x)
        self.assertEqual(x*q, x_mul_q)

    def test_log_exp(self):
        phi = 1.0
        n_tmp = [1.0,1.0,1.0]
        n = np.array(n_tmp, dtype=np.float16)/np.linalg.norm(np.array(n_tmp, dtype=np.float16))
        o = Quaternion([cos(phi/2), n[0]*sin(phi/2), n[1]*sin(phi/2), n[2]*sin(phi/2)])
        z_o = 2*PureQuaternion(n*phi/2)
        for i in range(4):
            self.assertLess((z_o - 2*o.log()).data[i], 0.001); self.assertGreater((z_o - 2*o.log()).data[i], -0.001)
            self.assertLess((o - (1/2*z_o).exp()).data[i], 0.001); self.assertGreater((o - 1/2*z_o.exp()).data[i], -0.001)


class TestPureQuaternionMethods(unittest.TestCase):

    def test_real(self):
        qp = PureQuaternion([1,2,3])
        self.assertEqual(qp.real, 0)

    def test_dot(self):
        qp1 = PureQuaternion([1,2,3])
        qp2 = PureQuaternion([1,4,9])
        self.assertEqual(qp1.dot(qp2), 1*1+2*4+3*9)
        self.assertEqual(qp2.dot(qp1), 1*1+2*4+3*9)
    
    def test_cross(self):
        qp1 = PureQuaternion([1,2,3])
        qp2 = PureQuaternion([1,4,9])
        qp1_cross_qp2 = PureQuaternion([6,-6,2])
        qp2_cross_qp1 = -qp1_cross_qp2
        self.assertEqual(qp1.cross(qp2), qp1_cross_qp2)
        self.assertEqual(qp2.cross(qp1), qp2_cross_qp1)

    # def test_mul_matrix(self):
    #     x = np.array([[1,2,3],[4,5,6],[7,8,9]],dtype=np.float16)
    #     q = PureQuaternion([10,20,30])
    #     x_mul_q = PureQuaternion([1*10+2*20+3*30, 4*10+5*20+6*30, 7*10+8*20+9*30])
    #     print(isinstance(x, type(np.array(1))) and x.shape == (3,3))
    #     print(x*q)
    #     print(x_mul_q)
    #     self.assertEqual(x*q, x_mul_q)

if __name__ == '__main__':
    unittest.main()
