import { GameApp } from './game/app';
import './styles.css';

const mount = (): void => {
  const container = document.getElementById('app');
  if (!container) throw new Error('Missing #app container');

  const app = new GameApp(container);
  app.start();

  // Expose for end-to-end tests and debugging. Harmless in production.
  (window as unknown as { gravityGolf: GameApp }).gravityGolf = app;

  document.body.classList.remove('loading');
};

if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', mount, { once: true });
} else {
  mount();
}
