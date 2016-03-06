//-----------------------------------------------
//
//	This file is part of the HamFramework for Siv3D.
//
//	Copyright (C) 2014-2016 Hamukun
//	Copyright (C) 2014-2016 Ryo Suzuki
//
//	Licensed under the MIT License.
//
//-----------------------------------------------

# pragma once
//# include "../Siv3D.hpp"
# include <Siv3D.hpp>

namespace ham
{
	enum class TransitionState
	{
		None,
		FadeIn,
		Active,
		FadeOut,
		FadeInOut,
	};

	template<class State, class Data> class LayerManager;

	template<class State, class Data> 
	class LayerBase : public std::enable_shared_from_this<LayerBase<State, Data>>, s3d::Uncopyable
	{
	public:	

		using Manager_t = LayerManager<State, Data>;

		using State_t = State;

		using Data_t = Data;

		virtual ~LayerBase() = default;

		void setData(Manager_t* pManager, const std::shared_ptr<Data>& data)
		{
			m_manager = pManager;

			m_data = data;
		}

		void setTransitionValue(int transitionTimeMillisec)
		{
			m_transitionTimeMillisec = transitionTimeMillisec;

			m_transitionState = TransitionState::FadeIn;

			m_stopwatch.restart();
		}

		s3d::Stopwatch& getStopwatch()
		{
			return m_stopwatch;
		}

		int32 getTransitionTimeMillisec() const
		{
			return m_transitionTimeMillisec;
		}

		void setTransitionState(const TransitionState& transitionState)
		{
			m_transitionState = transitionState;
		}

		const TransitionState& getTransitionState() const
		{
			return m_transitionState;
		}

		bool isDestroyed() const
		{
			return m_isDestroyed;
		}

		void setDestroyed(bool isDestroyed)
		{
			m_isDestroyed = isDestroyed;
		}

		virtual void init() {}

		virtual bool input() { return false; }

		virtual bool inputFadeIn(double) { return false; }

		virtual bool inputFadeOut(double) { return false; }

		virtual void updateFadeIn(double) {}

		virtual void update() = 0;

		virtual void updateFadeOut(double) {}

		virtual void draw() const = 0;

		virtual void drawFadeIn(double) const
		{
			draw();
		}

		virtual void drawFadeOut(double) const 
		{
			draw();
		}

	protected:

		std::shared_ptr<Data> m_data;

		bool pushLayer(const State& state, int transitionTimeMillisec = 200)
		{
			return m_manager->pushLayer(state, transitionTimeMillisec);
		}

		bool popThisLayer()
		{
			return m_manager->popLayer(shared_from_this());
		}

	private:

		Manager_t* m_manager = nullptr;

		s3d::Stopwatch m_stopwatch;

		int32 m_transitionTimeMillisec = 0;

		TransitionState m_transitionState = TransitionState::None;

		bool m_isDestroyed = false;
	};

	template<class State, class Data> class LayerManager
	{
	private:

		using Layer_t = std::shared_ptr<LayerBase<State, Data>>;

		using FactoryFunction_t = std::function<Layer_t()>;

		std::unordered_map<State, FactoryFunction_t> m_factories;

		std::shared_ptr<Data> m_data;

		Array<Layer_t> m_layers;

		Array<Layer_t> m_tLayers;

		s3d::Optional<State> m_first;

		bool m_error = false;

		template<class Type>
		std::shared_ptr<Type> MakeShared() const
		{
			return std::make_shared<Data>();
		}

		template<>
		std::shared_ptr<void> MakeShared() const
		{
			return std::shared_ptr<void>(nullptr);
		}

	public:

		using Layer = LayerBase<State, Data>;

		LayerManager()
			: m_data(MakeShared<Data>())
		{

		}

		LayerManager(const std::shared_ptr<Data>& data)
			: m_data(data)
		{

		}

		template<class Layer> bool add(const State& state)
		{
			if (m_factories.find(state) != m_factories.end())
			{
				return false;
			}

			m_factories.emplace(state, [&]()
			{
				auto m = std::make_shared<Layer>();

				m->setData(this, m_data);

				return m;
			});

			if (!m_first)
			{
				m_first = state;
			}

			return true;
		}

		bool init(const State& state)
		{
			if (m_layers.size() != 0)
			{
				m_layers.clear();
			}

			auto it = m_factories.find(state);

			if (it == m_factories.end())
			{
				return false;
			}

			auto newLayer = it->second();

			newLayer->init();

			m_layers.push_back(newLayer);

			return true;
		}

		bool input()
		{
			for (auto i = m_layers.rbegin(); i != m_layers.rend(); ++i)
			{
				auto& layer = *i;

				const auto state = layer->getTransitionState();

				if (state == TransitionState::FadeIn || state == TransitionState::Active)
				{
					if (layer->input())
					{
						break;
					}
				}			
			}

			if (m_tLayers.size() > 0)
			{
				m_layers.insert(m_layers.end(), m_tLayers.begin(), m_tLayers.end());

				m_tLayers.clear();
			}			

			return true;
		}

		bool update()
		{
			if (m_layers.size() == 0 && !init(m_first.value()))
			{
				return false;
			}

			for (auto& layer : m_layers)
			{
				const int32 elapsed = layer->getStopwatch().ms();

				switch (layer->getTransitionState())
				{
				case TransitionState::FadeIn:
					layer->updateFadeIn(static_cast<double>(elapsed) / layer->getTransitionTimeMillisec());
					if (elapsed > layer->getTransitionTimeMillisec())
					{
						layer->getStopwatch().reset();
						layer->setTransitionState(TransitionState::Active);
					}
					break;
				case TransitionState::Active:
					layer->update();
					break;
				case TransitionState::FadeOut:
					layer->updateFadeOut(static_cast<double>(elapsed) / layer->getTransitionTimeMillisec());
					if (elapsed > layer->getTransitionTimeMillisec())
					{
						layer->setDestroyed(true);
					}
					break;
				default:
					break;
				}
			}

			Erase_if(m_layers, [&](const Layer_t& layer)
			{
				return layer->isDestroyed();
			});

			if (m_tLayers.size() > 0)
			{
				m_layers.insert(m_layers.end(), m_tLayers.begin(), m_tLayers.end());

				m_tLayers.clear();
			}

			return true;
		}

		void draw() const
		{
			for (const auto& layer : m_layers)
			{
				const int32 elapsed = layer->getStopwatch().ms();

				switch (layer->getTransitionState())
				{
				case TransitionState::FadeIn:
					layer->drawFadeIn(static_cast<double>(elapsed) / layer->getTransitionTimeMillisec());
					break;
				case TransitionState::Active:
					layer->draw();
					break;
				case TransitionState::FadeOut:
					layer->drawFadeOut(static_cast<double>(elapsed) / layer->getTransitionTimeMillisec());
					break;
				default:
					break;
				}
			}
		}

		bool inputAndUpdateAndDraw()
		{
			if (!input())
			{
				return false;
			}

			if (!update())
			{
				return false;
			}

			draw();

			return true;
		}

		std::shared_ptr<Data> get()
		{
			return m_data;
		}

		bool pushLayer(const State& state, int transitionTimeMillisec = 200)
		{
			if (m_factories.find(state) == m_factories.end())
			{
				return false;
			}

			auto newLayer = m_factories[state]();

			newLayer->setTransitionValue(transitionTimeMillisec);

			m_tLayers.push_back(newLayer);

			newLayer->init();

			return true;
		}

		bool popLayer(const std::shared_ptr<Layer>& layer)
		{
			auto it = std::find(m_layers.begin(), m_layers.end(), layer);

			if (it == m_layers.end())
			{
				return false;
			}

			(*it)->setTransitionState(TransitionState::FadeOut);
			(*it)->getStopwatch().restart();

			return true;
		}
	};
}
