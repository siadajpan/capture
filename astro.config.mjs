// @ts-check
import { defineConfig } from 'astro/config';

// https://astro.build/config
export default defineConfig({
  site: 'https://capture.me.uk',
  server: {
    host: true,
    port: 4321
  }
});
