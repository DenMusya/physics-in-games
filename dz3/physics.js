export function rectangleInertia(mass, width, height) {
  return (mass * (width * width + height * height)) / 12;
}
