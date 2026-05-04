export class SceneManager {
  constructor() {
    this.scenes = new Map();
    this.current = null;
    this.currentName = "";
  }

  addScene(name, scene) {
    this.scenes.set(name, scene);
    scene.init();
  }

  setScene(name) {
    const next = this.scenes.get(name);
    if (!next) return;

    this.currentName = name;
    this.current = next;
    this.current.reset();
  }

  update(dt) {
    if (this.current) this.current.update(dt);
  }

  render() {
    if (this.current) this.current.render();
  }

  keyPressed(key) {
    if (this.current?.keyPressed) this.current.keyPressed(key);
  }

  mousePressed(x, y) {
    if (this.current?.mousePressed) this.current.mousePressed(x, y);
  }

  mouseDragged(x, y) {
    if (this.current?.mouseDragged) this.current.mouseDragged(x, y);
  }

  mouseReleased(x, y) {
    if (this.current?.mouseReleased) this.current.mouseReleased(x, y);
  }
}
