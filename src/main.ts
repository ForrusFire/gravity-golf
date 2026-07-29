import { GameApp } from './game/app';
import { ALL_LEVELS, levelById } from './game/levels';
import './styles.css';

const mount = (): void => {
  const container = document.getElementById('app');
  if (!container) throw new Error('Missing #app container');

  const app = new GameApp(container);
  app.start();

  // Exposed for end-to-end tests and the screenshot tool. Read-only helpers
  // plus the app itself; harmless in production.
  Object.assign(app as object, {
    levelIds: () => ALL_LEVELS.map((level) => level.id),
    debugPlay: (id: string) => {
      const level = levelById(id);
      if (level) app.playLevel(level);
    },
  });
  (window as unknown as { gravityGolf: GameApp }).gravityGolf = app;

  document.body.classList.remove('loading');
};

if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', mount, { once: true });
} else {
  mount();
}
