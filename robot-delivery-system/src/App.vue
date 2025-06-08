<script setup lang="ts">
import { RouterView } from 'vue-router'
</script>

<template>
  <div id="app">
    <RouterView />
  </div>
</template>

<style>
* {
  /* margin: 0!important;
  padding: 0!important; */
  box-sizing: border-box;
}

#app {
  font-family: 'Helvetica Neue', Helvetica, Arial, sans-serif;
  color: #2c3e50!important;
  min-height: 100vh;
}

body {
  /* margin: 0!important;
  padding: 0!important; */
}
</style>
