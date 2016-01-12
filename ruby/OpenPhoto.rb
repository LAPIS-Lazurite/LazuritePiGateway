require "starruby"
include StarRuby

texture = Texture.load("/home/pi/photo/camera1.jpeg")

game.run(320,240) do |game|
	game.screen.clear
	game.screen.render_texture(texture, 8, 8)
end
