import { createRouter, createWebHistory } from 'vue-router'
import MainDashboard from '../views/MainDashboard.vue'

const router = createRouter({
  history: createWebHistory(import.meta.env.BASE_URL),
  routes: [
    {
      path: '/',
      name: 'dashboard',
      component: MainDashboard,
    },
    {
      path: '/map',
      name: 'map',
      component: () => import('../views/MapDashboard.vue'),
    },
    {
      path: '/call',
      name: 'call',
      component: () => import('../views/CallRobot.vue'),
    },
    {
      path: '/send',
      name: 'send',
      component: () => import('../views/SendPackage.vue'),
    },
    {
      path: '/receive',
      name: 'receive',
      component: () => import('../views/ReceivePackage.vue'),
    },
    {
      path: '/test',
      name: 'apiTest',
      component: () => import('../views/ApiTestView.vue'),
    },
  ],
})

export default router
